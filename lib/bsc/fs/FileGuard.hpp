// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef FS_FILECLOSEGUARD_H
#define FS_FILECLOSEGUARD_H

#include <FS.h>

/** The FileGuard uses the RAII programming techinque to open a file on instantiation of the FileGuard object
 *  and closes it on scope exit. The file does not have to be closed manually.
 *  It can be used as follows:
 *  @code
 *  {
 *    fs::FileGuard fileGuard(fs, "index.htm", "r"); // RAII: Open the file in the file system fs
 *    if (fileGuard.isFileOpen())
 *    {
 *      const char* text = "Example text";
 *      fileGuard.getFile().write(text, sizeof(text));
 *    }
 *  } // Note: File is closed in scope exit!
 *
 *  @endcode
*/
namespace fs
{

class FileGuard
{
  // Do not allow to copy this class
  public:
  FileGuard(const FileGuard&) = delete;
  FileGuard& operator=(const FileGuard&) = delete;

  public:
  /** Dtor opens a file on the given file path.
   *  @param[in] fs The files system on which the file must be opened
   *  @param[in] path The file path
   *  @param[in] mode Open mode
   *  @param[in] create On true the file will be created if it does not exist.
   *
   *  @note For parameter details see documentation of fs::FS.
  */
  FileGuard(fs::FS& fs, const String& path, const char* mode = FILE_READ, const bool create = false)
    : _file(std::move(fs.open(path, mode, create)))
  {}

  /**
   * Dtor closes the file.
  */
  ~FileGuard()
  {
    _file.close();
  }

  /**
   * Returns the reference to the opened file.
  */
  File& getFile() noexcept
  {
    return _file;
  }

  /**
   * Returns true, if the file was successfully opened and it is a file not a directory.
  */
  bool isFile() const
  {
    // We have to const_cast the _file, be const isDirectory is not const qualified, but it should.
    return (true == _file) && (!const_cast<File&>(_file).isDirectory());
  }

  private:
  fs::File _file;
};

} // namespace fs

#endif // FS_FILECLOSEGUARD_H