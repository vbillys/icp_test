#include "file_writer.h"

namespace mcl_3d {
namespace io {

StreamFileWriter::StreamFileWriter(const string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {}

StreamFileWriter::~StreamFileWriter() {}

bool StreamFileWriter::Write(const char* const data, const size_t len) {
  if (out_.bad()) {
    return false;
  }
  out_.write(data, len);
  return !out_.bad();
}

bool StreamFileWriter::Close() {
  if (out_.bad()) {
    return false;
  }
  out_.close();
  return !out_.bad();
}

bool StreamFileWriter::WriteHeader(const char* const data, const size_t len) {
  if (out_.bad()) {
    return false;
  }
  out_.flush();
  out_.seekp(0);
  return Write(data, len);
}

}  // namespace io
}  // namespace mcl_3d
