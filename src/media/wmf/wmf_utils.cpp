#include "wmf_utils.h"

#include <cstring>
#include <limits>

namespace minirtc::wmf {

CodecThread::CodecThread() {
  std::promise<HRESULT> started;
  auto result = started.get_future();
  thread_ = std::thread([this, &started] {
    const HRESULT com = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    const HRESULT mf = SUCCEEDED(com) ? MFStartup(MF_VERSION) : com;
    started.set_value(mf);
    for (;;) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
        if (tasks_.empty() && stopping_) break;
        task = std::move(tasks_.front());
        tasks_.pop();
      }
      task();
    }
    if (SUCCEEDED(mf)) MFShutdown();
    if (SUCCEEDED(com)) CoUninitialize();
  });
  status_ = result.get();
}

CodecThread::~CodecThread() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  ready_.notify_one();
  thread_.join();
}

CallbackThread::CallbackThread() {
  thread_ = std::thread([this] {
    for (;;) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
        if (tasks_.empty() && stopping_) break;
        task = std::move(tasks_.front());
        tasks_.pop();
        ++active_;
      }
      task();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        --active_;
        if (tasks_.empty() && active_ == 0) {
          idle_.notify_all();
        }
      }
    }
  });
}

CallbackThread::~CallbackThread() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  ready_.notify_one();
  if (thread_.joinable()) thread_.join();
}

void CallbackThread::Post(std::function<void()> task) {
  if (!task) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) return;
    tasks_.push(std::move(task));
  }
  ready_.notify_one();
}

void CallbackThread::Flush() {
  std::unique_lock<std::mutex> lock(mutex_);
  idle_.wait(lock, [this] { return tasks_.empty() && active_ == 0; });
}

HRESULT CreateSample(const uint8_t* data, size_t size, LONGLONG time,
                     LONGLONG duration, ComPtr<IMFSample>& sample) {
  sample.Reset();
  if (!data || !size || size > std::numeric_limits<DWORD>::max())
    return E_INVALIDARG;
  ComPtr<IMFMediaBuffer> buffer;
  HRESULT hr = MFCreateMemoryBuffer(static_cast<DWORD>(size), &buffer);
  if (FAILED(hr)) return hr;
  BYTE* destination = nullptr;
  hr = buffer->Lock(&destination, nullptr, nullptr);
  if (FAILED(hr)) return hr;
  std::memcpy(destination, data, size);
  buffer->Unlock();
  hr = buffer->SetCurrentLength(static_cast<DWORD>(size));
  if (FAILED(hr)) return hr;
  hr = MFCreateSample(&sample);
  if (FAILED(hr)) return hr;
  hr = sample->AddBuffer(buffer.Get());
  if (FAILED(hr)) return hr;
  hr = sample->SetSampleTime(time);
  if (FAILED(hr)) return hr;
  return sample->SetSampleDuration(duration);
}

HRESULT ReadOutput(IMFTransform* transform, DWORD stream_id,
                   ReusableOutputSample& cache, ComPtr<IMFSample>& sample) {
  sample.Reset();
  MFT_OUTPUT_STREAM_INFO info{};
  HRESULT hr = transform->GetOutputStreamInfo(stream_id, &info);
  if (FAILED(hr)) return hr;
  ComPtr<IMFSample> supplied;
  if (!(info.dwFlags & (MFT_OUTPUT_STREAM_PROVIDES_SAMPLES |
                        MFT_OUTPUT_STREAM_CAN_PROVIDE_SAMPLES))) {
    const DWORD needed_size = info.cbSize ? info.cbSize : 1;
    const DWORD needed_align = info.cbAlignment ? info.cbAlignment - 1 : 0;
    if (!cache.sample || !cache.buffer || cache.size < needed_size ||
        cache.alignment != needed_align) {
      cache.Reset();
      hr = MFCreateSample(&cache.sample);
      if (FAILED(hr)) return hr;
      hr = MFCreateAlignedMemoryBuffer(needed_size, needed_align,
                                       &cache.buffer);
      if (FAILED(hr)) {
        cache.Reset();
        return hr;
      }
      hr = cache.sample->AddBuffer(cache.buffer.Get());
      if (FAILED(hr)) {
        cache.Reset();
        return hr;
      }
      cache.size = needed_size;
      cache.alignment = needed_align;
    } else {
      DWORD count = 0;
      if (FAILED(cache.sample->GetBufferCount(&count)) || count != 1) {
        cache.sample->RemoveAllBuffers();
        hr = cache.sample->AddBuffer(cache.buffer.Get());
        if (FAILED(hr)) return hr;
      }
      cache.sample->DeleteAllItems();
    }
    hr = cache.buffer->SetCurrentLength(0);
    if (FAILED(hr)) return hr;
    supplied = cache.sample;
  }
  MFT_OUTPUT_DATA_BUFFER output{};
  output.dwStreamID = stream_id;
  output.pSample = supplied.Get();
  DWORD status = 0;
  hr = transform->ProcessOutput(0, 1, &output, &status);
  if (output.pEvents) output.pEvents->Release();
  // A sample supplied by the caller is not AddRef'd by ProcessOutput.
  if (output.pSample == supplied.Get())
    sample = supplied;
  else
    sample.Attach(output.pSample);
  return hr;
}

HRESULT ReadOutput(IMFTransform* transform, DWORD stream_id,
                   ComPtr<IMFSample>& sample) {
  ReusableOutputSample temp_cache;
  return ReadOutput(transform, stream_id, temp_cache, sample);
}

HRESULT ReadBytes(IMFSample* sample, std::vector<uint8_t>& bytes) {
  if (!sample) return E_INVALIDARG;
  ComPtr<IMFMediaBuffer> buffer;
  DWORD buffer_count = 0;
  HRESULT hr = sample->GetBufferCount(&buffer_count);
  if (SUCCEEDED(hr) && buffer_count == 1) {
    hr = sample->GetBufferByIndex(0, &buffer);
  } else {
    hr = sample->ConvertToContiguousBuffer(&buffer);
  }
  if (FAILED(hr)) return hr;
  BYTE* data = nullptr;
  DWORD size = 0;
  hr = buffer->Lock(&data, nullptr, &size);
  if (FAILED(hr)) return hr;
  bytes.assign(data, data + size);
  buffer->Unlock();
  return S_OK;
}

void ShutdownTransform(ComPtr<IMFTransform>& transform) {
  if (!transform) return;
  transform->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, 0);
  transform->ProcessMessage(MFT_MESSAGE_NOTIFY_END_OF_STREAM, 0);
  transform->ProcessMessage(MFT_MESSAGE_NOTIFY_END_STREAMING, 0);
  ComPtr<IMFShutdown> shutdown;
  if (SUCCEEDED(transform.As(&shutdown))) shutdown->Shutdown();
  transform.Reset();
}

std::vector<Nalu> AnnexBNalus(const uint8_t* data, size_t size) {
  std::vector<Nalu> result;
  if (!data) return result;
  size_t payload = size;
  for (size_t i = 0; i + 2 < size;) {
    size_t prefix = 0;
    if (data[i] == 0 && data[i + 1] == 0) {
      if (data[i + 2] == 1)
        prefix = 3;
      else if (i + 3 < size && data[i + 2] == 0 && data[i + 3] == 1)
        prefix = 4;
    }
    if (!prefix) {
      ++i;
      continue;
    }
    if (payload < i)
      result.push_back(
          {payload, i - payload, static_cast<uint8_t>(data[payload] & 0x1f)});
    payload = i + prefix;
    i = payload;
  }
  if (payload < size)
    result.push_back(
        {payload, size - payload, static_cast<uint8_t>(data[payload] & 0x1f)});
  return result;
}

bool HasNalu(const uint8_t* data, size_t size, uint8_t type) {
  for (const auto& nalu : AnnexBNalus(data, size))
    if (nalu.type == type) return true;
  return false;
}

std::vector<Nalu> AnnexBHevcNalus(const uint8_t* data, size_t size) {
  std::vector<Nalu> result;
  if (!data) return result;
  size_t payload = size;
  for (size_t i = 0; i + 2 < size;) {
    size_t prefix = 0;
    if (data[i] == 0 && data[i + 1] == 0) {
      if (data[i + 2] == 1)
        prefix = 3;
      else if (i + 3 < size && data[i + 2] == 0 && data[i + 3] == 1)
        prefix = 4;
    }
    if (!prefix) {
      ++i;
      continue;
    }
    if (payload + 1 < i) {
      result.push_back(
          {payload, i - payload,
           static_cast<uint8_t>((data[payload] >> 1) & 0x3Fu)});
    }
    payload = i + prefix;
    i = payload;
  }
  if (payload + 1 < size) {
    result.push_back(
        {payload, size - payload,
         static_cast<uint8_t>((data[payload] >> 1) & 0x3Fu)});
  }
  return result;
}

bool HasHevcNalu(const uint8_t* data, size_t size, uint8_t type) {
  for (const auto& nalu : AnnexBHevcNalus(data, size))
    if (nalu.type == type) return true;
  return false;
}

bool IsHevcKeyFrame(const uint8_t* data, size_t size) {
  for (const auto& nalu : AnnexBHevcNalus(data, size)) {
    if (nalu.type == 19 || nalu.type == 20 || nalu.type == 21) return true;
  }
  return false;
}
}  // namespace minirtc::wmf
