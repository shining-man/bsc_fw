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
  #include <LITTLEFS.h> 
#else
  #include <SPIFFS.h>
#endif 
#include <WebServer.h>

static const char * TAG = "WEB";

//void listDir(fs::FS &fs, const char * dirname, uint8_t levels);

String getContentType(WebServer *server, String filename)
{
  if (server->hasArg("download")) {
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

bool exists(bool fsIsSpiffs, String path)
{
  bool yes = false;
  File file;
  file = SPIFFS.open(path, "r");
  if(!file.isDirectory())
  {
    yes = true;
  }
  file.close();
  return yes;
}

bool handleFileRead(WebServer *server, bool fsIsSpiffs, String path)
{
  if(!SPIFFS.begin())
  {
    BSC_LOGE(TAG,"LITTLEFS Mount Failed");
  }

  if (path.endsWith("/"))
  {
    path += "index.htm";
  }
  String contentType = getContentType(server, path);
  String pathWithGz = path + ".gz";
  if (exists(fsIsSpiffs, pathWithGz) || exists(fsIsSpiffs, path))
  {
    /*if (exists(fsIsSpiffs, pathWithGz))
    {
      path += ".gz";
    }*/
    File file;
    file = SPIFFS.open(path, "r");
    //if(file==NULL) BSC_LOGI(TAG,"%s => null",path.c_str());
    server->streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}


void handleFileUpload(WebServer *server, bool fsIsSpiffs, String fileName) 
{
  static File fsUploadFile;
  HTTPUpload& upload = server->upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    //if (upload.filename.length() > 30)
    //{
    //  upload.filename = upload.filename.substring(upload.filename.length() - 30, upload.filename.length());  // Dateinamen auf 30 Zeichen kürzen
    //}
    upload.filename=fileName;
    BSC_LOGI(TAG,"handleFileUpload Name: /%s", upload.filename.c_str());
    fsUploadFile = SPIFFS.open("/" + server->urlDecode(upload.filename), "w");
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