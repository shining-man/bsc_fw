// Copyright (c) 2022 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "webUtility.h"
#include "defines.h"
#include "log.h"
#include <FS.h>
#ifdef USE_LittleFS
  #define SPIFFS LittleFS
  #include <LittleFS.h>
#else
  #include <SPIFFS.h>
#endif
#include <fs/FileGuard.hpp>
#include <WebServer.h>

namespace // anonymous - methods, variables, and types in this namespace are have file scope only
{

const char * TAG = "WEB";

//void listDir(fs::FS &fs, const char * dirname, uint8_t levels);

auto getContentType(WebServer &server, const String &filename)
{
  if (server.hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  }
  return "text/plain";
}
} // namespace anonymous

bool handleFileRead(fs::FS &fs, WebServer &server, [[maybe_unused]] bool fsIsSpiffs, const String &path)
{
  const String basePath = (path.endsWith("/")) ? (path + "index.htm") : path;
  [[maybe_unused]] const String pathWithGz = path + ".gz";

  fs::FileGuard fileGuard(fs, basePath, "r"); // RAII: Dtor of the guard closes the file on scope exit (method return)
  if (fileGuard.isFile())
  {
    // TODO MEJ: How shall we handle pathWithGz?
    /*if (exists(fsIsSpiffs, pathWithGz))
    {
      path += ".gz";
    }*/

    const String contentType {getContentType(server, fileGuard.getFile().name())};
    server.streamFile(fileGuard.getFile(), contentType);

    return true;
  }

  return false;
}

void handleFileUpload(WebServer &server, [[maybe_unused]] bool fsIsSpiffs, const String &fileName)
{
  static File fsUploadFile;
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    //if (upload.filename.length() > 30)
    //{
    //  upload.filename = upload.filename.substring(upload.filename.length() - 30, upload.filename.length());  // Dateinamen auf 30 Zeichen kürzen
    //}
    upload.filename = fileName;
    BSC_LOGI(TAG,"handleFileUpload Name: /%s", upload.filename.c_str());
    fsUploadFile = SPIFFS.open("/" + server.urlDecode(upload.filename), "w");
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    //BSC_LOGI(TAG,"handleFileUpload Data: %u\n", upload.currentSize);
    if(fsUploadFile) fsUploadFile.write(upload.buf, upload.currentSize);
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (fsUploadFile) fsUploadFile.close();
    //BSC_LOGI(TAG,"handleFileUpload Size: %u\n", upload.totalSize);
    BSC_LOGI(TAG,"handleFileUpload finish");
  }
}

/*void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    BSC_LOGI(TAG, "Listing directory: %s", dirname);

    File root = fs.open(dirname);
    if(!root){
        BSC_LOGI(TAG, "Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        BSC_LOGI(TAG, "Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            BSC_LOGI(TAG, "DIR=%s", file.name());
            time_t t= file.getLastWrite();
            struct tm * tmstruct = localtime(&t);
            BSC_LOGI(TAG, "  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d",(tmstruct->tm_year)+1900,( tmstruct->tm_mon)+1, tmstruct->tm_mday,tmstruct->tm_hour , tmstruct->tm_min, tmstruct->tm_sec);


            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            BSC_LOGI(TAG, "FILE=%s, size=%i", file.name(), file.size());

            time_t t= file.getLastWrite();
            struct tm * tmstruct = localtime(&t);
            BSC_LOGI(TAG, "  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d",(tmstruct->tm_year)+1900,( tmstruct->tm_mon)+1, tmstruct->tm_mday,tmstruct->tm_hour , tmstruct->tm_min, tmstruct->tm_sec);
        }
        file = root.openNextFile();
    }
}*/
