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

namespace // anonymous namespace - methods, variables, and types in this namespace have file scope only
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

void handleFileUpload(fs::FS &fs, WebServer &server, [[maybe_unused]] bool fsIsSpiffs, const String &fileName)
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
    fsUploadFile = fs.open("/" + server.urlDecode(upload.filename), "w");
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


bool performAuthentication(WebServer &server, WebSettings &ws)
{
  String bscName = ws.getStringFlash(ID_PARAM_BSC_USERNAME, 0);
  String bscPw = ws.getStringFlash(ID_PARAM_BSC_PASSWORD, 0);


  if (!server.authenticate(bscName.c_str(), bscPw.c_str()))
  {
    server.requestAuthentication();
    return false;
  }
  return true;
}

bool performAuthentication(WebServer *server, WebSettings *ws)
{
  if(server==nullptr) return false;
  if(ws==nullptr) return false;

  String bscName = ws->getStringFlash(ID_PARAM_BSC_USERNAME, 0);
  String bscPw = ws->getStringFlash(ID_PARAM_BSC_PASSWORD, 0);

  if (!server->authenticate(bscName.c_str(), bscPw.c_str()))
  {
    server->requestAuthentication();
    return false;
  }
  return true;
}

bool performAuthentication(WebServer *server, WebSettings &ws)
{
  if(server==nullptr) return false;

  String bscName = ws.getStringFlash(ID_PARAM_BSC_USERNAME, 0);
  String bscPw = ws.getStringFlash(ID_PARAM_BSC_PASSWORD, 0);

  if (!server->authenticate(bscName.c_str(), bscPw.c_str()))
  {
    server->requestAuthentication();
    return false;
  }
  return true;
}


//const char* loginForm = "<form method='post' action='/login'><input type='text' name='username' placeholder='Benutzername'><br><input type='password' name='password' placeholder='Passwort'><br><input type='submit' value='Einloggen'></form>";

/*bool performAuthentication(WebServer &server)
{
  BSC_LOGI(TAG,"performAuthentication");
  if (!server.authenticate(http_username, http_password))
  {
    BSC_LOGI(TAG,"performAuthentication OK");
    server.sendHeader("Location", "/login");
    server.send(301);
    return true;

    //server.requestAuthentication();
    //return false;
  }
  BSC_LOGI(TAG,"performAuthentication OK2");
  return true;
}

void handleLogin(WebServer &server) {
  // Überprüfen, ob die POST-Daten Benutzername und Passwort korrekt sind
  if (server.hasArg("username") && server.hasArg("password")) {
    if (server.arg("username") == http_username && server.arg("password") == http_password) {
      // Weiterleitung zur Hauptseite, wenn die Anmeldeinformationen korrekt sind
      server.sendHeader("Location", "/");
      server.send(301);
      return;
    }
  }
  // Falls Anmeldeinformationen nicht korrekt sind, wird die Loginseite erneut angezeigt
  server.send(200, "text/html", loginForm);
}

void handleLogout(WebServer &server) {
  // Lösche die HTTP Basic Authentication-Informationen im Client (Webbrowser)
  server.sendHeader("WWW-Authenticate", "Basic realm=\"Secure Area\"");
  server.send(401);
}*/


// Escaped einen String so, dass er sicher in HTML (insb. Formulare) angezeigt werden kann.
// Geeignet für Textknoten und Attributwerte.
// Für Attribute werden zusätzlich Zeilenumbrüche als &#10; kodiert.
String htmlEscape(const String& in, bool forAttribute) {
  String out;
  out.reserve(in.length() + 8); // grob, spart Reallocs

  for (size_t i = 0; i < in.length(); ++i) {
    char c = in[i];
    switch (c) {
      case '&':  out += F("&amp;");  break;
      case '<':  out += F("&lt;");   break;
      case '>':  out += F("&gt;");   break;
      case '"':  out += F("&quot;"); break;
      case '\'': out += F("&#39;");  break;  // &apos; geht auch, &#39; ist robuster
      case '\n':
        if (forAttribute) out += F("&#10;");  // Zeilenumbruch im value=""
        else out += '\n';
        break;
      case '\r':
        // CR in Attributen weglassen, sonst normal übernehmen
        if (!forAttribute) out += '\r';
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

String urlDecode(const String& in) {
  String out;
  out.reserve(in.length());

  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '+') { out += ' '; }
    else if (c == '%' && i + 2 < in.length()) {
      char h1 = in[i+1], h2 = in[i+2];
      auto hex = [](char x)->int {
        if (x>='0'&&x<='9') return x-'0';
        if (x>='A'&&x<='F') return x-'A'+10;
        if (x>='a'&&x<='f') return x-'a'+10;
        return -1;
      };
      int hi = hex(h1), lo = hex(h2);
      if (hi >= 0 && lo >= 0) {
        out += char((hi<<4) | lo);
        i += 2;
      } else {
        out += c; // ungültig, roh übernehmen
      }
    } else {
      out += c;
    }
  }
  return out;
}
