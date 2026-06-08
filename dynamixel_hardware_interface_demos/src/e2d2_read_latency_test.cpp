#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <errno.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sched.h>
#include <sys/epoll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include "dynamixel_sdk/group_bulk_read.h"
#include "dynamixel_sdk/group_fast_bulk_read.h"
#include "dynamixel_sdk/group_fast_sync_read.h"
#include "dynamixel_sdk/group_sync_read.h"
#include "dynamixel_sdk/packet_handler.h"
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/port_handler_e2d2.h"

namespace
{

using Clock = std::chrono::steady_clock;
using Micros = std::chrono::duration<double, std::micro>;

constexpr int kDefaultPresentPositionAddress = 132;
constexpr int kDefaultPresentPositionLength = 4;
constexpr double kProtocolVersion = 2.0;

struct Options
{
  std::string ip = "192.168.100.1";
  std::string transport = "tcp";
  std::string mode = "single";
  std::string read_method = "basic";
  std::vector<int> channels{1};
  int dxl_id = 1;
  int baudrate = 6000000;
  int address = kDefaultPresentPositionAddress;
  int length = kDefaultPresentPositionLength;
  double hz = 1000.0;
  double duration_sec = 10.0;
  int warmup = 50;
  int open_retries = 50;
  int open_retry_ms = 20;
  int rt_priority = 0;
  int cpu_base = -1;
  bool lock_memory = false;
  bool resync_on_miss = false;
  std::string csv_path;
};

struct Sample
{
  double latency_us = 0.0;
  double wake_jitter_us = 0.0;
  int comm_result = 0;
  uint8_t dxl_error = 0;
  bool data_available = true;
  bool deadline_miss = false;
};

struct Stats
{
  int channel = 0;
  bool setup_ok = false;
  std::string setup_error;
  uint16_t model_number = 0;
  std::vector<Sample> samples;
};

struct SharedStart
{
  std::mutex mutex;
  std::condition_variable cv;
  int ready = 0;
  bool go = false;
  bool stop = false;
  Clock::time_point start_time;
};

void notifySetupFailure(SharedStart * start_state)
{
  std::lock_guard<std::mutex> lock(start_state->mutex);
  start_state->stop = true;
  start_state->cv.notify_all();
}

std::string makePortName(const Options & opt, int channel)
{
  std::ostringstream oss;
  oss << "e2d2" << (opt.transport == "udp" ? "udp" : "tcp")
      << ":" << opt.ip << ":" << (5000 + channel);
  return oss.str();
}

std::string commResultToString(int result)
{
  const char * text =
    dynamixel::PacketHandler::getPacketHandler(kProtocolVersion)->getTxRxResult(result);
  return text == nullptr ? "unknown" : text;
}

void printUsage(const char * argv0)
{
  std::cout
    << "Usage:\n"
    << "  " << argv0 << " --mode single --channel 1 --duration 10 --hz 1000 --read basic\n"
    << "  " << argv0 << " --mode parallel --channels 1-6 --duration 10 --hz 1000 --read basic\n\n"
    << "  " << argv0 << " --mode epoll --channels 1-6 --duration 10 --hz 1000 --read basic\n\n"
    << "  " << argv0 << " --mode epoll-sdk --channels 1-6 --duration 10 --hz 1000 --read basic\n\n"
    << "Options:\n"
    << "  --ip ADDR              E2D2 IP address. Default: 192.168.100.1\n"
    << "  --transport tcp|udp    E2D2 bridge transport. Default: tcp\n"
    << "  --mode single|parallel|epoll|epoll-sdk Test mode. Default: single\n"
    << "  --channel N            Single channel number. Default: 1\n"
    << "  --channels LIST        Parallel channels, e.g. 1-6 or 1,2,3. Default: 1\n"
    << "  --id N                 DYNAMIXEL ID. Default: 1\n"
    << "  --baudrate N           Bridge baudrate. Default: 6000000\n"
    << "  --addr N               Read start address. Default: 132\n"
    << "  --len N                Read byte length. Default: 4\n"
    << "  --duration SEC         Timed loop duration. Default: 10\n"
    << "  --hz HZ                Loop frequency. Default: 1000\n"
    << "  --read METHOD          basic, sync, bulk, fast-sync, fast-bulk. Default: basic\n"
    << "  --warmup N             Untimed warmup reads per channel. Default: 50\n"
    << "  --open-retries N       Port open retry count. Default: 50\n"
    << "  --open-retry-ms N      Delay between port open retries. Default: 20\n"
    << "  --rt-priority N        Use SCHED_FIFO priority N when permitted. Default: 0\n"
    << "  --cpu-base N           Pin channel threads to CPUs N, N+1, ... Default: disabled\n"
    << "  --lock-memory          Call mlockall(MCL_CURRENT | MCL_FUTURE)\n"
    << "  --resync-on-miss       Reset the next release time after a deadline miss\n"
    << "  --csv PATH             Write per-sample latency data as CSV\n"
    << "  --help                 Show this help\n";
}

bool parseInt(const std::string & text, int * value)
{
  char * end = nullptr;
  long parsed = std::strtol(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  *value = static_cast<int>(parsed);
  return true;
}

bool parseDouble(const std::string & text, double * value)
{
  char * end = nullptr;
  double parsed = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

std::vector<int> parseChannels(const std::string & text)
{
  std::vector<int> channels;
  const auto dash = text.find('-');
  if (dash != std::string::npos) {
    int first = 0;
    int last = 0;
    if (!parseInt(text.substr(0, dash), &first) ||
      !parseInt(text.substr(dash + 1), &last) || first > last)
    {
      return {};
    }
    for (int channel = first; channel <= last; ++channel) {
      channels.push_back(channel);
    }
    return channels;
  }

  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    int channel = 0;
    if (!parseInt(item, &channel)) {
      return {};
    }
    channels.push_back(channel);
  }
  return channels;
}

bool parseArgs(int argc, char ** argv, Options * opt)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto needValue = [&](const std::string & name) -> const char * {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--ip") {
      const char * value = needValue(arg);
      if (!value) return false;
      opt->ip = value;
    } else if (arg == "--transport") {
      const char * value = needValue(arg);
      if (!value) return false;
      opt->transport = value;
    } else if (arg == "--mode") {
      const char * value = needValue(arg);
      if (!value) return false;
      opt->mode = value;
    } else if (arg == "--read") {
      const char * value = needValue(arg);
      if (!value) return false;
      opt->read_method = value;
    } else if (arg == "--channel") {
      const char * value = needValue(arg);
      if (!value) return false;
      int channel = 0;
      if (!parseInt(value, &channel)) return false;
      opt->channels = {channel};
    } else if (arg == "--channels") {
      const char * value = needValue(arg);
      if (!value) return false;
      opt->channels = parseChannels(value);
    } else if (arg == "--id") {
      const char * value = needValue(arg);
      if (!value || !parseInt(value, &opt->dxl_id)) return false;
    } else if (arg == "--baudrate") {
      const char * value = needValue(arg);
      if (!value || !parseInt(value, &opt->baudrate)) return false;
    } else if (arg == "--addr") {
      const char * value = needValue(arg);
      if (!value || !parseInt(value, &opt->address)) return false;
    } else if (arg == "--len") {
      const char * value = needValue(arg);
      if (!value || !parseInt(value, &opt->length)) return false;
    } else if (arg == "--duration") {
      const char * value = needValue(arg);
      if (!value || !parseDouble(value, &opt->duration_sec)) return false;
    } else if (arg == "--hz") {
      const char * value = needValue(arg);
      if (!value || !parseDouble(value, &opt->hz)) return false;
    } else if (arg == "--warmup") {
      const char * value = needValue(arg);
      if (!value || !parseInt(value, &opt->warmup)) return false;
    } else if (arg == "--open-retries") {
      const char * value = needValue(arg);
      if (!value || !parseInt(value, &opt->open_retries)) return false;
    } else if (arg == "--open-retry-ms") {
      const char * value = needValue(arg);
      if (!value || !parseInt(value, &opt->open_retry_ms)) return false;
    } else if (arg == "--rt-priority") {
      const char * value = needValue(arg);
      if (!value || !parseInt(value, &opt->rt_priority)) return false;
    } else if (arg == "--cpu-base") {
      const char * value = needValue(arg);
      if (!value || !parseInt(value, &opt->cpu_base)) return false;
    } else if (arg == "--lock-memory") {
      opt->lock_memory = true;
    } else if (arg == "--resync-on-miss") {
      opt->resync_on_miss = true;
    } else if (arg == "--csv") {
      const char * value = needValue(arg);
      if (!value) return false;
      opt->csv_path = value;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return false;
    }
  }

  if (opt->mode != "single" && opt->mode != "parallel" &&
    opt->mode != "epoll" && opt->mode != "epoll-sdk")
  {
    std::cerr << "--mode must be single, parallel, epoll, or epoll-sdk\n";
    return false;
  }
  if (opt->mode == "single" && opt->channels.size() != 1) {
    std::cerr << "--mode single requires exactly one channel\n";
    return false;
  }
  if ((opt->mode == "epoll" || opt->mode == "epoll-sdk") && opt->read_method != "basic") {
    std::cerr << "--mode " << opt->mode << " currently supports --read basic only\n";
    return false;
  }
  if (opt->transport != "tcp" && opt->transport != "udp") {
    std::cerr << "--transport must be tcp or udp\n";
    return false;
  }
  if (opt->read_method != "basic" && opt->read_method != "sync" &&
    opt->read_method != "bulk" && opt->read_method != "fast-sync" &&
    opt->read_method != "fast-bulk")
  {
    std::cerr << "--read must be basic, sync, bulk, fast-sync, or fast-bulk\n";
    return false;
  }
  if (opt->channels.empty()) {
    std::cerr << "No valid channels were selected\n";
    return false;
  }
  for (const int channel : opt->channels) {
    if (channel < 1 || channel > 6) {
      std::cerr << "E2D2 channel must be 1..6: " << channel << "\n";
      return false;
    }
  }
  if (opt->dxl_id < 0 || opt->dxl_id > 252) {
    std::cerr << "--id must be 0..252\n";
    return false;
  }
  if (opt->length <= 0 || opt->length > 255 || opt->address < 0 || opt->address > 65535) {
    std::cerr << "--addr/--len out of range\n";
    return false;
  }
  if (opt->hz <= 0.0 || opt->duration_sec <= 0.0 || opt->baudrate <= 0) {
    std::cerr << "--hz, --duration, and --baudrate must be positive\n";
    return false;
  }
  if (opt->open_retries < 1 || opt->open_retry_ms < 0) {
    std::cerr << "--open-retries must be >= 1 and --open-retry-ms must be >= 0\n";
    return false;
  }
  return true;
}

void trySetRealtime(int priority, int cpu)
{
  if (cpu >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    const int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (rc != 0) {
      std::cerr << "warning: failed to pin thread to CPU " << cpu << ": "
                << std::strerror(rc) << "\n";
    }
  }

  if (priority > 0) {
    sched_param param;
    std::memset(&param, 0, sizeof(param));
    param.sched_priority = priority;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (rc != 0) {
      std::cerr << "warning: failed to set SCHED_FIFO priority " << priority
                << ": " << std::strerror(rc) << "\n";
    }
  }
}

uint32_t decodeLittleEndian(const std::vector<uint8_t> & data)
{
  uint32_t value = 0;
  const size_t bytes = std::min<size_t>(data.size(), sizeof(value));
  for (size_t i = 0; i < bytes; ++i) {
    value |= static_cast<uint32_t>(data[i]) << (8U * i);
  }
  return value;
}

bool openPortWithRetry(const Options & opt, dynamixel::PortHandler * port)
{
  for (int attempt = 0; attempt < opt.open_retries; ++attempt) {
    if (port->openPort()) {
      return true;
    }
    if (opt.open_retry_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(opt.open_retry_ms));
    }
  }
  return false;
}

bool setBaudRateSerialized(dynamixel::PortHandler * port, int baudrate)
{
  static std::mutex baudrate_mutex;
  std::lock_guard<std::mutex> lock(baudrate_mutex);
  return port->setBaudRate(baudrate);
}

struct ReadContext
{
  std::vector<uint8_t> data;
  std::unique_ptr<dynamixel::GroupSyncRead> sync;
  std::unique_ptr<dynamixel::GroupBulkRead> bulk;
  std::unique_ptr<dynamixel::GroupFastSyncRead> fast_sync;
  std::unique_ptr<dynamixel::GroupFastBulkRead> fast_bulk;
};

bool initReadContext(
  const Options & opt,
  dynamixel::PortHandler * port,
  dynamixel::PacketHandler * packet,
  ReadContext * ctx)
{
  ctx->data.assign(static_cast<size_t>(opt.length), 0);

  const uint8_t id = static_cast<uint8_t>(opt.dxl_id);
  const auto address = static_cast<uint16_t>(opt.address);
  const auto length = static_cast<uint16_t>(opt.length);

  if (opt.read_method == "sync") {
    ctx->sync.reset(new dynamixel::GroupSyncRead(port, packet, address, length));
    return ctx->sync->addParam(id);
  }
  if (opt.read_method == "bulk") {
    ctx->bulk.reset(new dynamixel::GroupBulkRead(port, packet));
    return ctx->bulk->addParam(id, address, length);
  }
  if (opt.read_method == "fast-sync") {
    ctx->fast_sync.reset(new dynamixel::GroupFastSyncRead(port, packet, address, length));
    return ctx->fast_sync->addParam(id);
  }
  if (opt.read_method == "fast-bulk") {
    ctx->fast_bulk.reset(new dynamixel::GroupFastBulkRead(port, packet));
    return ctx->fast_bulk->addParam(id, address, length);
  }
  return true;
}

int doRead(
  const Options & opt,
  dynamixel::PortHandler * port,
  dynamixel::PacketHandler * packet,
  ReadContext * ctx,
  uint32_t * value,
  uint8_t * dxl_error,
  bool * data_available)
{
  const uint8_t id = static_cast<uint8_t>(opt.dxl_id);
  const auto address = static_cast<uint16_t>(opt.address);
  const auto length = static_cast<uint16_t>(opt.length);
  *dxl_error = 0;
  *data_available = true;
  *value = 0;

  if (opt.read_method == "basic") {
    const int rc = packet->readTxRx(
      port, id, address, length, ctx->data.data(), dxl_error);
    *value = decodeLittleEndian(ctx->data);
    return rc;
  }

  if (opt.read_method == "sync") {
    const int rc = ctx->sync->txRxPacket();
    *data_available = ctx->sync->isAvailable(id, address, length);
    if (*data_available) {
      *value = ctx->sync->getData(id, address, length);
    }
    ctx->sync->getError(id, dxl_error);
    return rc;
  }

  if (opt.read_method == "bulk") {
    const int rc = ctx->bulk->txRxPacket();
    *data_available = ctx->bulk->isAvailable(id, address, length);
    if (*data_available) {
      *value = ctx->bulk->getData(id, address, length);
    }
    ctx->bulk->getError(id, dxl_error);
    return rc;
  }

  if (opt.read_method == "fast-sync") {
    const int rc = ctx->fast_sync->txRxPacket();
    *data_available = ctx->fast_sync->isAvailable(id, address, length);
    if (*data_available) {
      *value = ctx->fast_sync->getData(id, address, length);
    }
    ctx->fast_sync->getError(id, dxl_error);
    return rc;
  }

  const int rc = ctx->fast_bulk->txRxPacket();
  *data_available = ctx->fast_bulk->isAvailable(id, address, length);
  if (*data_available) {
    *value = ctx->fast_bulk->getData(id, address, length);
  }
  ctx->fast_bulk->getError(id, dxl_error);
  return rc;
}

Stats runChannel(
  const Options & opt,
  int channel,
  int thread_index,
  SharedStart * start_state)
{
  Stats stats;
  stats.channel = channel;

  trySetRealtime(
    opt.rt_priority,
    opt.cpu_base >= 0 ? opt.cpu_base + thread_index : -1);

  const std::string port_name = makePortName(opt, channel);
  std::unique_ptr<dynamixel::PortHandler> port(
    dynamixel::PortHandler::getPortHandler(port_name.c_str()));
  dynamixel::PacketHandler * packet =
    dynamixel::PacketHandler::getPacketHandler(kProtocolVersion);

  if (!port || !openPortWithRetry(opt, port.get())) {
    stats.setup_error = "failed to open " + port_name + " after " +
      std::to_string(opt.open_retries) + " attempts";
    notifySetupFailure(start_state);
    return stats;
  }
  if (!setBaudRateSerialized(port.get(), opt.baudrate)) {
    stats.setup_error = "failed to set baudrate on " + port_name;
    notifySetupFailure(start_state);
    return stats;
  }

  uint8_t ping_error = 0;
  const int ping_rc = packet->ping(
    port.get(), static_cast<uint8_t>(opt.dxl_id), &stats.model_number, &ping_error);
  if (ping_rc != COMM_SUCCESS || stats.model_number == 0) {
    std::ostringstream oss;
    oss << "ping failed on " << port_name << " rc=" << ping_rc
        << " (" << commResultToString(ping_rc) << ")"
        << " model=" << stats.model_number
        << " error=" << static_cast<int>(ping_error);
    stats.setup_error = oss.str();
    notifySetupFailure(start_state);
    return stats;
  }

  ReadContext read_context;
  if (!initReadContext(opt, port.get(), packet, &read_context)) {
    stats.setup_error = "failed to initialize read context for " + opt.read_method;
    notifySetupFailure(start_state);
    return stats;
  }

  for (int i = 0; i < opt.warmup; ++i) {
    uint32_t value = 0;
    uint8_t dxl_error = 0;
    bool data_available = true;
    doRead(opt, port.get(), packet, &read_context, &value, &dxl_error, &data_available);
  }

  {
    std::unique_lock<std::mutex> lock(start_state->mutex);
    stats.setup_ok = true;
    start_state->ready += 1;
    start_state->cv.notify_all();
    start_state->cv.wait(lock, [&]() {return start_state->go || start_state->stop;});
    if (start_state->stop) {
      stats.setup_error = "aborted before start";
      return stats;
    }
  }

  const auto period =
    std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / opt.hz));
  const int iterations = static_cast<int>(std::llround(opt.duration_sec * opt.hz));
  stats.samples.reserve(static_cast<size_t>(iterations));

  auto release_time = start_state->start_time;
  for (int i = 0; i < iterations; ++i) {
    std::this_thread::sleep_until(release_time);

    const auto start = Clock::now();
    const double wake_jitter_us =
      std::chrono::duration_cast<Micros>(start - release_time).count();

    uint32_t value = 0;
    uint8_t dxl_error = 0;
    bool data_available = true;
    const int rc = doRead(
      opt, port.get(), packet, &read_context, &value, &dxl_error, &data_available);
    (void)value;

    const auto finish = Clock::now();
    const auto deadline = release_time + period;

    Sample sample;
    sample.latency_us = std::chrono::duration_cast<Micros>(finish - start).count();
    sample.wake_jitter_us = wake_jitter_us > 0.0 ? wake_jitter_us : 0.0;
    sample.comm_result = rc;
    sample.dxl_error = dxl_error;
    sample.data_available = data_available;
    sample.deadline_miss = finish > deadline;
    stats.samples.push_back(sample);

    release_time = (opt.resync_on_miss && sample.deadline_miss) ?
      finish + period : release_time + period;
  }

  return stats;
}

uint16_t updateDxlCrc(const uint8_t * data, size_t size)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x8005);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

std::vector<uint8_t> makeBasicReadPacket(const Options & opt)
{
  std::vector<uint8_t> packet(14, 0);
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = 0xFD;
  packet[3] = 0x00;
  packet[4] = static_cast<uint8_t>(opt.dxl_id);
  packet[5] = 0x07;
  packet[6] = 0x00;
  packet[7] = 0x02;  // READ
  packet[8] = static_cast<uint8_t>(opt.address & 0xFF);
  packet[9] = static_cast<uint8_t>((opt.address >> 8) & 0xFF);
  packet[10] = static_cast<uint8_t>(opt.length & 0xFF);
  packet[11] = static_cast<uint8_t>((opt.length >> 8) & 0xFF);

  const uint16_t crc = updateDxlCrc(packet.data(), packet.size() - 2);
  packet[12] = static_cast<uint8_t>(crc & 0xFF);
  packet[13] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  return packet;
}

bool setNonblockingFd(int fd)
{
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

int openConnectedSocket(const Options & opt, int channel, std::string * error)
{
  const bool udp = opt.transport == "udp";
  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(5000 + channel));
  if (inet_pton(AF_INET, opt.ip.c_str(), &addr.sin_addr) != 1) {
    *error = "invalid IPv4 address: " + opt.ip;
    return -1;
  }

  for (int attempt = 0; attempt < opt.open_retries; ++attempt) {
    const int fd = socket(AF_INET, udp ? SOCK_DGRAM : SOCK_STREAM, 0);
    if (fd < 0) {
      *error = std::string("socket failed: ") + std::strerror(errno);
      return -1;
    }

    if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
      if (!udp) {
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      }

      if (!setNonblockingFd(fd)) {
        *error = std::string("fcntl O_NONBLOCK failed: ") + std::strerror(errno);
        close(fd);
        return -1;
      }
      return fd;
    }

    *error = std::string("connect failed: ") + std::strerror(errno);
    close(fd);
    if (opt.open_retry_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(opt.open_retry_ms));
    }
  }

  return -1;
}

void drainSocket(int fd)
{
  uint8_t buffer[512];
  while (true) {
    const ssize_t n = recv(fd, buffer, sizeof(buffer), MSG_DONTWAIT);
    if (n > 0) {
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    break;
  }
}

bool prepareChannelWithSdk(const Options & opt, int channel, Stats * stats)
{
  stats->channel = channel;

  const std::string port_name = makePortName(opt, channel);
  std::unique_ptr<dynamixel::PortHandler> port(
    dynamixel::PortHandler::getPortHandler(port_name.c_str()));
  dynamixel::PacketHandler * packet =
    dynamixel::PacketHandler::getPacketHandler(kProtocolVersion);

  if (!port || !openPortWithRetry(opt, port.get())) {
    stats->setup_error = "failed to open " + port_name + " after " +
      std::to_string(opt.open_retries) + " attempts";
    return false;
  }
  if (!setBaudRateSerialized(port.get(), opt.baudrate)) {
    stats->setup_error = "failed to set baudrate on " + port_name;
    return false;
  }

  uint8_t ping_error = 0;
  const int ping_rc = packet->ping(
    port.get(), static_cast<uint8_t>(opt.dxl_id), &stats->model_number, &ping_error);
  if (ping_rc != COMM_SUCCESS || stats->model_number == 0) {
    std::ostringstream oss;
    oss << "ping failed on " << port_name << " rc=" << ping_rc
        << " (" << commResultToString(ping_rc) << ")"
        << " model=" << stats->model_number
        << " error=" << static_cast<int>(ping_error);
    stats->setup_error = oss.str();
    return false;
  }

  ReadContext read_context;
  if (!initReadContext(opt, port.get(), packet, &read_context)) {
    stats->setup_error = "failed to initialize read context for " + opt.read_method;
    return false;
  }
  for (int i = 0; i < opt.warmup; ++i) {
    uint32_t value = 0;
    uint8_t dxl_error = 0;
    bool data_available = true;
    doRead(opt, port.get(), packet, &read_context, &value, &dxl_error, &data_available);
  }

  stats->setup_ok = true;
  return true;
}

struct EpollChannel
{
  int fd = -1;
  int epoll_index = 0;
  int channel = 0;
  std::vector<uint8_t> rx_buffer;
  bool pending = false;
  size_t sample_index = 0;
  Clock::time_point release_time;
  Clock::time_point send_time;
  double wake_jitter_us = 0.0;
  Stats stats;
};

int tryExtractStatus(
  const Options & opt,
  EpollChannel * channel,
  uint8_t * dxl_error,
  bool * data_available)
{
  *dxl_error = 0;
  *data_available = false;

  auto & buffer = channel->rx_buffer;
  while (buffer.size() >= 7) {
    size_t header = buffer.size();
    for (size_t i = 0; i + 3 < buffer.size(); ++i) {
      if (buffer[i] == 0xFF && buffer[i + 1] == 0xFF &&
        buffer[i + 2] == 0xFD && buffer[i + 3] != 0xFD)
      {
        header = i;
        break;
      }
    }
    if (header == buffer.size()) {
      buffer.erase(buffer.begin(), buffer.end() - 3);
      return COMM_RX_FAIL;
    }
    if (header > 0) {
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(header));
    }
    if (buffer.size() < 7) {
      return COMM_RX_FAIL;
    }

    const uint16_t packet_length =
      static_cast<uint16_t>(buffer[5] | (static_cast<uint16_t>(buffer[6]) << 8));
    const size_t total_length = static_cast<size_t>(packet_length) + 7;
    if (total_length < 11 || total_length > 4096) {
      buffer.erase(buffer.begin());
      return COMM_RX_CORRUPT;
    }
    if (buffer.size() < total_length) {
      return COMM_RX_FAIL;
    }

    std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + total_length);
    buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(total_length));

    const uint16_t received_crc =
      static_cast<uint16_t>(packet[total_length - 2] |
      (static_cast<uint16_t>(packet[total_length - 1]) << 8));
    if (updateDxlCrc(packet.data(), total_length - 2) != received_crc) {
      return COMM_RX_CORRUPT;
    }
    if (packet[4] != static_cast<uint8_t>(opt.dxl_id) || packet[7] != 0x55) {
      return COMM_RX_CORRUPT;
    }
    if (total_length < static_cast<size_t>(11 + opt.length)) {
      return COMM_RX_CORRUPT;
    }

    *dxl_error = packet[8];
    *data_available = true;
    return COMM_SUCCESS;
  }

  return COMM_RX_FAIL;
}

void completeEpollSample(
  const Options & opt,
  EpollChannel * channel,
  int comm_result,
  uint8_t dxl_error,
  bool data_available,
  Clock::time_point finish_time)
{
  Sample sample;
  sample.latency_us =
    std::chrono::duration_cast<Micros>(finish_time - channel->send_time).count();
  sample.wake_jitter_us = channel->wake_jitter_us;
  sample.comm_result = comm_result;
  sample.dxl_error = dxl_error;
  sample.data_available = data_available;
  sample.deadline_miss =
    finish_time > channel->release_time + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / opt.hz));

  channel->stats.samples.push_back(sample);
  channel->pending = false;
}

std::vector<Stats> runEpollMode(const Options & opt)
{
  trySetRealtime(opt.rt_priority, opt.cpu_base);

  std::vector<EpollChannel> channels(opt.channels.size());
  for (size_t i = 0; i < opt.channels.size(); ++i) {
    channels[i].epoll_index = static_cast<int>(i);
    channels[i].channel = opt.channels[i];
    if (!prepareChannelWithSdk(opt, opt.channels[i], &channels[i].stats)) {
      std::vector<Stats> failed;
      for (const auto & channel : channels) {
        if (channel.stats.channel != 0) {
          failed.push_back(channel.stats);
        }
      }
      return failed;
    }

    std::string socket_error;
    channels[i].fd = openConnectedSocket(opt, opt.channels[i], &socket_error);
    if (channels[i].fd < 0) {
      channels[i].stats.setup_error = "epoll socket open failed on channel " +
        std::to_string(opt.channels[i]) + ": " + socket_error;
      std::vector<Stats> failed;
      for (const auto & channel : channels) {
        if (channel.stats.channel != 0) {
          failed.push_back(channel.stats);
        }
      }
      return failed;
    }
    drainSocket(channels[i].fd);
  }

  const int epoll_fd = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd < 0) {
    for (auto & channel : channels) {
      channel.stats.setup_error = std::string("epoll_create1 failed: ") + std::strerror(errno);
    }
  } else {
    for (size_t i = 0; i < channels.size(); ++i) {
      epoll_event event;
      std::memset(&event, 0, sizeof(event));
      event.events = EPOLLIN | EPOLLERR | EPOLLHUP;
      event.data.u32 = static_cast<uint32_t>(i);
      if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, channels[i].fd, &event) != 0) {
        channels[i].stats.setup_error = std::string("epoll_ctl failed: ") + std::strerror(errno);
      }
    }
  }

  const std::vector<uint8_t> read_packet = makeBasicReadPacket(opt);
  const auto period =
    std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / opt.hz));
  const int iterations = static_cast<int>(std::llround(opt.duration_sec * opt.hz));
  for (auto & channel : channels) {
    channel.stats.samples.reserve(static_cast<size_t>(iterations));
  }

  if (epoll_fd >= 0 &&
    std::all_of(channels.begin(), channels.end(), [](const EpollChannel & channel) {
      return channel.stats.setup_error.empty();
    }))
  {
    auto release_time = Clock::now() + std::chrono::milliseconds(250);
    std::this_thread::sleep_until(release_time);

    std::vector<epoll_event> events(channels.size());
    for (int iteration = 0; iteration < iterations; ++iteration) {
      std::this_thread::sleep_until(release_time);

      int pending_count = 0;
      for (auto & channel : channels) {
        drainSocket(channel.fd);
        channel.rx_buffer.clear();
        const auto send_time = Clock::now();
        const ssize_t sent = send(channel.fd, read_packet.data(), read_packet.size(), MSG_NOSIGNAL);
        channel.release_time = release_time;
        channel.send_time = send_time;
        channel.wake_jitter_us =
          std::max(0.0, std::chrono::duration_cast<Micros>(send_time - release_time).count());
        channel.pending = true;
        channel.sample_index = static_cast<size_t>(iteration);

        if (sent != static_cast<ssize_t>(read_packet.size())) {
          completeEpollSample(
            opt, &channel, COMM_TX_FAIL, 0, false, Clock::now());
        } else {
          pending_count += 1;
        }
      }

      const auto timeout_at = Clock::now() + std::chrono::milliseconds(20);
      while (pending_count > 0) {
        const auto now = Clock::now();
        if (now >= timeout_at) {
          for (auto & channel : channels) {
            if (channel.pending) {
              completeEpollSample(opt, &channel, COMM_RX_TIMEOUT, 0, false, now);
              pending_count -= 1;
            }
          }
          break;
        }

        const int timeout_ms = std::max(
          0,
          static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(timeout_at - now).count()));
        const int ready = epoll_wait(
          epoll_fd, events.data(), static_cast<int>(events.size()), timeout_ms);
        if (ready < 0) {
          if (errno == EINTR) {
            continue;
          }
          for (auto & channel : channels) {
            if (channel.pending) {
              completeEpollSample(opt, &channel, COMM_RX_FAIL, 0, false, Clock::now());
              pending_count -= 1;
            }
          }
          break;
        }
        if (ready == 0) {
          continue;
        }

        for (int event_index = 0; event_index < ready; ++event_index) {
          const uint32_t channel_index = events[event_index].data.u32;
          if (channel_index >= channels.size()) {
            continue;
          }
          auto & channel = channels[channel_index];
          if (!channel.pending) {
            continue;
          }
          if ((events[event_index].events & (EPOLLERR | EPOLLHUP)) != 0) {
            completeEpollSample(opt, &channel, COMM_RX_FAIL, 0, false, Clock::now());
            pending_count -= 1;
            continue;
          }

          bool socket_closed = false;
          while (true) {
            uint8_t buffer[512];
            const ssize_t n = recv(channel.fd, buffer, sizeof(buffer), MSG_DONTWAIT);
            if (n > 0) {
              channel.rx_buffer.insert(channel.rx_buffer.end(), buffer, buffer + n);
              continue;
            }
            if (n == 0) {
              socket_closed = true;
            } else if (errno == EINTR) {
              continue;
            }
            break;
          }

          if (socket_closed) {
            completeEpollSample(opt, &channel, COMM_RX_FAIL, 0, false, Clock::now());
            pending_count -= 1;
            continue;
          }

          uint8_t dxl_error = 0;
          bool data_available = false;
          const int parse_result =
            tryExtractStatus(opt, &channel, &dxl_error, &data_available);
          if (parse_result == COMM_SUCCESS || parse_result == COMM_RX_CORRUPT) {
            completeEpollSample(
              opt, &channel, parse_result, dxl_error, data_available, Clock::now());
            pending_count -= 1;
          }
        }
      }

      bool iteration_missed = false;
      for (const auto & channel : channels) {
        if (!channel.stats.samples.empty() && channel.stats.samples.back().deadline_miss) {
          iteration_missed = true;
          break;
        }
      }
      release_time = (opt.resync_on_miss && iteration_missed) ?
        Clock::now() + period : release_time + period;
    }
  }

  if (epoll_fd >= 0) {
    close(epoll_fd);
  }

  std::vector<Stats> results;
  results.reserve(channels.size());
  for (auto & channel : channels) {
    if (channel.fd >= 0) {
      close(channel.fd);
      channel.fd = -1;
    }
    results.push_back(std::move(channel.stats));
  }
  return results;
}

struct EpollSdkChannel
{
  int fd = -1;
  int epoll_index = 0;
  int channel = 0;
  std::unique_ptr<dynamixel::PortHandler> port;
  std::vector<uint8_t> data;
  bool pending = false;
  size_t sample_index = 0;
  Clock::time_point release_time;
  Clock::time_point send_time;
  double wake_jitter_us = 0.0;
  Stats stats;
};

bool prepareEpollSdkChannel(const Options & opt, int channel, EpollSdkChannel * epoll_channel)
{
  epoll_channel->channel = channel;
  epoll_channel->stats.channel = channel;
  epoll_channel->data.assign(static_cast<size_t>(opt.length), 0);

  const std::string port_name = makePortName(opt, channel);
  epoll_channel->port.reset(dynamixel::PortHandler::getPortHandler(port_name.c_str()));
  dynamixel::PacketHandler * packet =
    dynamixel::PacketHandler::getPacketHandler(kProtocolVersion);

  if (!epoll_channel->port || !openPortWithRetry(opt, epoll_channel->port.get())) {
    epoll_channel->stats.setup_error = "failed to open " + port_name + " after " +
      std::to_string(opt.open_retries) + " attempts";
    return false;
  }
  if (!setBaudRateSerialized(epoll_channel->port.get(), opt.baudrate)) {
    epoll_channel->stats.setup_error = "failed to set baudrate on " + port_name;
    return false;
  }

  auto * bridge =
    dynamic_cast<dynamixel::PortHandlerE2D2Bridge *>(epoll_channel->port.get());
  if (bridge == nullptr || bridge->getSocketFd() < 0) {
    epoll_channel->stats.setup_error =
      "failed to access E2D2 bridge socket fd for " + port_name;
    return false;
  }
  epoll_channel->fd = bridge->getSocketFd();

  uint8_t ping_error = 0;
  const int ping_rc = packet->ping(
    epoll_channel->port.get(),
    static_cast<uint8_t>(opt.dxl_id),
    &epoll_channel->stats.model_number,
    &ping_error);
  if (ping_rc != COMM_SUCCESS || epoll_channel->stats.model_number == 0) {
    std::ostringstream oss;
    oss << "ping failed on " << port_name << " rc=" << ping_rc
        << " (" << commResultToString(ping_rc) << ")"
        << " model=" << epoll_channel->stats.model_number
        << " error=" << static_cast<int>(ping_error);
    epoll_channel->stats.setup_error = oss.str();
    return false;
  }

  ReadContext read_context;
  if (!initReadContext(opt, epoll_channel->port.get(), packet, &read_context)) {
    epoll_channel->stats.setup_error = "failed to initialize read context for " + opt.read_method;
    return false;
  }
  for (int i = 0; i < opt.warmup; ++i) {
    uint32_t value = 0;
    uint8_t dxl_error = 0;
    bool data_available = true;
    doRead(
      opt,
      epoll_channel->port.get(),
      packet,
      &read_context,
      &value,
      &dxl_error,
      &data_available);
  }

  epoll_channel->port->clearPort();
  epoll_channel->stats.setup_ok = true;
  return true;
}

void completeEpollSdkSample(
  const Options & opt,
  EpollSdkChannel * channel,
  int comm_result,
  uint8_t dxl_error,
  bool data_available,
  Clock::time_point finish_time)
{
  Sample sample;
  sample.latency_us =
    std::chrono::duration_cast<Micros>(finish_time - channel->send_time).count();
  sample.wake_jitter_us = channel->wake_jitter_us;
  sample.comm_result = comm_result;
  sample.dxl_error = dxl_error;
  sample.data_available = data_available;
  sample.deadline_miss =
    finish_time > channel->release_time + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / opt.hz));

  channel->stats.samples.push_back(sample);
  channel->pending = false;
}

std::vector<Stats> runEpollSdkMode(const Options & opt)
{
  trySetRealtime(opt.rt_priority, opt.cpu_base);

  std::vector<EpollSdkChannel> channels(opt.channels.size());
  for (size_t i = 0; i < opt.channels.size(); ++i) {
    channels[i].epoll_index = static_cast<int>(i);
    if (!prepareEpollSdkChannel(opt, opt.channels[i], &channels[i])) {
      std::vector<Stats> failed;
      for (auto & channel : channels) {
        if (channel.stats.channel != 0) {
          failed.push_back(std::move(channel.stats));
        }
      }
      return failed;
    }
  }

  const int epoll_fd = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd < 0) {
    for (auto & channel : channels) {
      channel.stats.setup_error = std::string("epoll_create1 failed: ") + std::strerror(errno);
    }
  } else {
    for (size_t i = 0; i < channels.size(); ++i) {
      epoll_event event;
      std::memset(&event, 0, sizeof(event));
      event.events = EPOLLIN | EPOLLERR | EPOLLHUP;
      event.data.u32 = static_cast<uint32_t>(i);
      if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, channels[i].fd, &event) != 0) {
        channels[i].stats.setup_error = std::string("epoll_ctl failed: ") + std::strerror(errno);
      }
    }
  }

  dynamixel::PacketHandler * packet =
    dynamixel::PacketHandler::getPacketHandler(kProtocolVersion);
  const auto period =
    std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / opt.hz));
  const int iterations = static_cast<int>(std::llround(opt.duration_sec * opt.hz));
  for (auto & channel : channels) {
    channel.stats.samples.reserve(static_cast<size_t>(iterations));
  }

  if (epoll_fd >= 0 &&
    std::all_of(channels.begin(), channels.end(), [](const EpollSdkChannel & channel) {
      return channel.stats.setup_error.empty();
    }))
  {
    auto release_time = Clock::now() + std::chrono::milliseconds(250);
    std::this_thread::sleep_until(release_time);

    std::vector<epoll_event> events(channels.size());
    for (int iteration = 0; iteration < iterations; ++iteration) {
      std::this_thread::sleep_until(release_time);

      int pending_count = 0;
      for (auto & channel : channels) {
        std::fill(channel.data.begin(), channel.data.end(), 0);
        const auto send_time = Clock::now();
        channel.release_time = release_time;
        channel.send_time = send_time;
        channel.wake_jitter_us =
          std::max(0.0, std::chrono::duration_cast<Micros>(send_time - release_time).count());
        channel.pending = true;
        channel.sample_index = static_cast<size_t>(iteration);

        const int tx_result = packet->readTx(
          channel.port.get(),
          static_cast<uint8_t>(opt.dxl_id),
          static_cast<uint16_t>(opt.address),
          static_cast<uint16_t>(opt.length));
        if (tx_result != COMM_SUCCESS) {
          channel.port->is_using_ = false;
          completeEpollSdkSample(opt, &channel, tx_result, 0, false, Clock::now());
        } else {
          pending_count += 1;
        }
      }

      const auto timeout_at = Clock::now() + std::chrono::milliseconds(20);
      while (pending_count > 0) {
        const auto now = Clock::now();
        if (now >= timeout_at) {
          for (auto & channel : channels) {
            if (channel.pending) {
              channel.port->is_using_ = false;
              channel.port->clearPort();
              completeEpollSdkSample(opt, &channel, COMM_RX_TIMEOUT, 0, false, now);
              pending_count -= 1;
            }
          }
          break;
        }

        const int timeout_ms = std::max(
          0,
          static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(timeout_at - now).count()));
        const int ready = epoll_wait(
          epoll_fd, events.data(), static_cast<int>(events.size()), timeout_ms);
        if (ready < 0) {
          if (errno == EINTR) {
            continue;
          }
          for (auto & channel : channels) {
            if (channel.pending) {
              channel.port->is_using_ = false;
              completeEpollSdkSample(opt, &channel, COMM_RX_FAIL, 0, false, Clock::now());
              pending_count -= 1;
            }
          }
          break;
        }
        if (ready == 0) {
          continue;
        }

        for (int event_index = 0; event_index < ready; ++event_index) {
          const uint32_t channel_index = events[event_index].data.u32;
          if (channel_index >= channels.size()) {
            continue;
          }
          auto & channel = channels[channel_index];
          if (!channel.pending) {
            continue;
          }
          if ((events[event_index].events & (EPOLLERR | EPOLLHUP)) != 0) {
            channel.port->is_using_ = false;
            completeEpollSdkSample(opt, &channel, COMM_RX_FAIL, 0, false, Clock::now());
            pending_count -= 1;
            continue;
          }

          uint8_t dxl_error = 0;
          const int rx_result = packet->readRx(
            channel.port.get(),
            static_cast<uint8_t>(opt.dxl_id),
            static_cast<uint16_t>(opt.length),
            channel.data.data(),
            &dxl_error);
          completeEpollSdkSample(
            opt,
            &channel,
            rx_result,
            dxl_error,
            rx_result == COMM_SUCCESS,
            Clock::now());
          pending_count -= 1;
        }
      }

      bool iteration_missed = false;
      for (const auto & channel : channels) {
        if (!channel.stats.samples.empty() && channel.stats.samples.back().deadline_miss) {
          iteration_missed = true;
          break;
        }
      }
      release_time = (opt.resync_on_miss && iteration_missed) ?
        Clock::now() + period : release_time + period;
    }
  }

  if (epoll_fd >= 0) {
    close(epoll_fd);
  }

  std::vector<Stats> results;
  results.reserve(channels.size());
  for (auto & channel : channels) {
    if (channel.port) {
      channel.port->closePort();
    }
    results.push_back(std::move(channel.stats));
  }
  return results;
}

double percentile(std::vector<double> values, double p)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double rank = (p / 100.0) * static_cast<double>(values.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(rank));
  const size_t hi = static_cast<size_t>(std::ceil(rank));
  if (lo == hi) {
    return values[lo];
  }
  const double weight = rank - static_cast<double>(lo);
  return values[lo] * (1.0 - weight) + values[hi] * weight;
}

struct Summary
{
  size_t samples = 0;
  double avg_latency_us = 0.0;
  double p95_latency_us = 0.0;
  double p99_latency_us = 0.0;
  double max_latency_us = 0.0;
  double avg_wake_jitter_us = 0.0;
  double p99_wake_jitter_us = 0.0;
  double max_wake_jitter_us = 0.0;
  size_t deadline_misses = 0;
  size_t comm_failures = 0;
  size_t dxl_errors = 0;
  size_t data_unavailable = 0;
  std::map<int, size_t> comm_result_counts;
  std::map<int, size_t> dxl_error_counts;
};

Summary summarize(const std::vector<Sample> & samples)
{
  Summary summary;
  summary.samples = samples.size();
  if (samples.empty()) {
    return summary;
  }

  std::vector<double> latencies;
  std::vector<double> wake_jitters;
  latencies.reserve(samples.size());
  wake_jitters.reserve(samples.size());

  double latency_sum = 0.0;
  double jitter_sum = 0.0;
  for (const auto & sample : samples) {
    latencies.push_back(sample.latency_us);
    wake_jitters.push_back(sample.wake_jitter_us);
    latency_sum += sample.latency_us;
    jitter_sum += sample.wake_jitter_us;
    summary.max_latency_us = std::max(summary.max_latency_us, sample.latency_us);
    summary.max_wake_jitter_us = std::max(summary.max_wake_jitter_us, sample.wake_jitter_us);
    if (sample.deadline_miss) {
      summary.deadline_misses += 1;
    }
    if (sample.comm_result != COMM_SUCCESS) {
      summary.comm_failures += 1;
      summary.comm_result_counts[sample.comm_result] += 1;
    }
    if (sample.dxl_error != 0) {
      summary.dxl_errors += 1;
      summary.dxl_error_counts[static_cast<int>(sample.dxl_error)] += 1;
    }
    if (!sample.data_available) {
      summary.data_unavailable += 1;
    }
  }

  summary.avg_latency_us = latency_sum / static_cast<double>(samples.size());
  summary.p95_latency_us = percentile(latencies, 95.0);
  summary.p99_latency_us = percentile(latencies, 99.0);
  summary.avg_wake_jitter_us = jitter_sum / static_cast<double>(samples.size());
  summary.p99_wake_jitter_us = percentile(wake_jitters, 99.0);
  return summary;
}

void printSummaryLine(
  const std::string & label,
  uint16_t model,
  const Summary & summary)
{
  std::cout << std::fixed << std::setprecision(2)
            << label
            << " model=" << model
            << " samples=" << summary.samples
            << " avg_us=" << summary.avg_latency_us
            << " p95_us=" << summary.p95_latency_us
            << " p99_us=" << summary.p99_latency_us
            << " max_us=" << summary.max_latency_us
            << " deadline_miss=" << summary.deadline_misses
            << " comm_fail=" << summary.comm_failures
            << " dxl_error=" << summary.dxl_errors
            << " unavailable=" << summary.data_unavailable
            << " wake_avg_us=" << summary.avg_wake_jitter_us
            << " wake_p99_us=" << summary.p99_wake_jitter_us
            << " wake_max_us=" << summary.max_wake_jitter_us
            << "\n";

  if (!summary.comm_result_counts.empty()) {
    std::cout << "  comm_results:";
    for (const auto & item : summary.comm_result_counts) {
      std::cout << " " << item.first << "(" << commResultToString(item.first)
                << ")=" << item.second;
    }
    std::cout << "\n";
  }

  if (!summary.dxl_error_counts.empty()) {
    std::cout << "  dxl_errors:";
    for (const auto & item : summary.dxl_error_counts) {
      std::cout << " " << item.first << "=" << item.second;
    }
    std::cout << "\n";
  }
}

bool writeCsv(const std::string & path, const std::vector<Stats> & results)
{
  std::ofstream out(path);
  if (!out.is_open()) {
    std::cerr << "failed to open CSV output: " << path << "\n";
    return false;
  }

  out << "channel,index,latency_us,wake_jitter_us,comm_result,dxl_error,"
      << "data_available,deadline_miss\n";
  out << std::fixed << std::setprecision(3);

  for (const auto & result : results) {
    for (size_t i = 0; i < result.samples.size(); ++i) {
      const auto & sample = result.samples[i];
      out << result.channel << ","
          << i << ","
          << sample.latency_us << ","
          << sample.wake_jitter_us << ","
          << sample.comm_result << ","
          << static_cast<int>(sample.dxl_error) << ","
          << (sample.data_available ? 1 : 0) << ","
          << (sample.deadline_miss ? 1 : 0) << "\n";
    }
  }

  if (!out.good()) {
    std::cerr << "failed while writing CSV output: " << path << "\n";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  if (!parseArgs(argc, argv, &opt)) {
    printUsage(argv[0]);
    return 2;
  }

  if (opt.lock_memory) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      std::cerr << "warning: mlockall failed: " << std::strerror(errno) << "\n";
    }
  }

  std::cout << "E2D2 read latency test"
            << " mode=" << opt.mode
            << " read=" << opt.read_method
            << " ip=" << opt.ip
            << " transport=" << opt.transport
            << " id=" << opt.dxl_id
            << " addr=" << opt.address
            << " len=" << opt.length
            << " hz=" << opt.hz
            << " duration_sec=" << opt.duration_sec
            << " baudrate=" << opt.baudrate
            << "\n";

  std::vector<Stats> results;
  if (opt.mode == "epoll") {
    results = runEpollMode(opt);
  } else if (opt.mode == "epoll-sdk") {
    results = runEpollSdkMode(opt);
  } else {
    SharedStart start_state;
    start_state.start_time = Clock::now() + std::chrono::milliseconds(250);

    results.resize(opt.channels.size());
    std::vector<std::thread> threads;
    threads.reserve(opt.channels.size());

    for (size_t i = 0; i < opt.channels.size(); ++i) {
      threads.emplace_back([&, i]() {
        results[i] = runChannel(opt, opt.channels[i], static_cast<int>(i), &start_state);
      });
    }

    {
      std::unique_lock<std::mutex> lock(start_state.mutex);
      start_state.cv.wait(lock, [&]() {
        const bool all_ready = start_state.ready == static_cast<int>(opt.channels.size());
        return all_ready || start_state.stop;
      });

      if (start_state.ready != static_cast<int>(opt.channels.size())) {
        start_state.stop = true;
        start_state.cv.notify_all();
      } else {
        start_state.start_time = Clock::now() + std::chrono::milliseconds(250);
        start_state.go = true;
        start_state.cv.notify_all();
      }
    }

    for (auto & thread : threads) {
      thread.join();
    }
  }

  bool ok = true;
  std::vector<Sample> aggregate_samples;
  for (const auto & result : results) {
    if (!result.setup_error.empty()) {
      ok = false;
      std::cout << "channel=" << result.channel << " setup_error="
                << result.setup_error << "\n";
      continue;
    }

    const Summary summary = summarize(result.samples);
    printSummaryLine("channel=" + std::to_string(result.channel), result.model_number, summary);
    aggregate_samples.insert(
      aggregate_samples.end(), result.samples.begin(), result.samples.end());

    if (summary.comm_failures != 0 || summary.data_unavailable != 0) {
      ok = false;
    }
  }

  if (results.size() > 1 && !aggregate_samples.empty()) {
    printSummaryLine("aggregate", 0, summarize(aggregate_samples));
  }

  if (!opt.csv_path.empty()) {
    const bool csv_ok = writeCsv(opt.csv_path, results);
    std::cout << "csv=" << opt.csv_path << " write=" << (csv_ok ? "ok" : "failed") << "\n";
    ok = ok && csv_ok;
  }

  return ok ? 0 : 1;
}
