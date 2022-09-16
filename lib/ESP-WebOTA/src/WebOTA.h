#include <Arduino.h>
#include <WebServer.h>

class WebOTA {
	public:
		int init(WebServer *server, const char *path);

	private:
		void delay(WebServer *server, unsigned int ms);
		int add_http_routes(WebServer *server, const char *path);
		
		bool init_has_run;
		String get_ota_html();
		long max_sketch_size();
};

extern WebOTA webota;
