#include "http_service.h"

#include "sws/server_http.hpp"
#include "sws/utility.hpp"

#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <fstream>
#include <vector>
#ifdef HAVE_OPENSSL
#include "crypto.hpp"
#endif

using namespace std;
using namespace boost::property_tree;

using HttpServer = SimpleWeb::Server<SimpleWeb::HTTP>;

void handleGet(std::string resource, std::shared_ptr<SimpleWeb::Server<SimpleWeb::HTTP>::Response> response, std::shared_ptr<SimpleWeb::Server<SimpleWeb::HTTP>::Request> request);

void handleGet(std::string resource, std::shared_ptr<SimpleWeb::Server<SimpleWeb::HTTP>::Response> response, std::shared_ptr<SimpleWeb::Server<SimpleWeb::HTTP>::Request> request) {
	try {
		std::map<std::string, std::string> requestMap;

		auto query_fields = request->parse_query_string();
		for (auto& field : query_fields)
		{
			std::cout << resource << " " << field.first << ": " << field.second << std::endl;
			requestMap[field.first] = field.second;
		}

		ptree res_json;
		res_json.put("success", true);

		std::map<std::string, std::string> ctrlMap;
		if (resource.compare("record") == 0) {
			ctrlMap["record"] = requestMap["action"];
		}

		//ctrl_callback(camera, ctrlMap);

		res_json.put("action", resource + requestMap["action"]);

		std::stringstream ss;
		boost::property_tree::json_parser::write_json(ss, res_json);

		*response << "HTTP/1.1 200 OK\r\n"
			<< "Content-Length: " << ss.str().length() << "\r\n\r\n"
			<< ss.str();
	}
	catch (const exception& e) {
		*response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << strlen(e.what()) << "\r\n\r\n"
			<< e.what();
	}
};

struct http_service {
	HttpServer server;
	int m_http_port;
};

static http_service* singleton_service;

void start_http_server() {

	singleton_service = new http_service();
	
	singleton_service->m_http_port = 8080;

	singleton_service->server.config.port = 8080;

	singleton_service->server.resource["^/record$"]["POST"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request) {
		try {
			ptree pt;
			read_json(request->content, pt);

			string type = pt.get<string>("type");

			if (type.compare("start") == 0) {
				
				//startRecording
				std::cout << " record:start" << std::endl;
			}
			else if (type.compare("stop") == 0) {
				//stopRecording
				std::cout << " record:stop" << std::endl;
			}

			std::cout << " record:end" << std::endl;

			ptree res_json;
			res_json.put("type", type);
			std::stringstream ss;
			boost::property_tree::json_parser::write_json(ss, res_json);

			*response << "HTTP/1.1 200 OK\r\n"
				<< "Content-Length: " << ss.str().length() << "\r\n\r\n"
				<< ss.str();
		}
		catch (const exception& e) {
			*response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << strlen(e.what()) << "\r\n\r\n"
				<< e.what();
		}
	};
	
	singleton_service->server.default_resource["GET"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request) {
		try {
			auto web_root_path = boost::filesystem::canonical("/home/vsemi/dev/anti_tailgating/web");
			auto path = boost::filesystem::canonical(web_root_path / request->path);
			
			if (distance(web_root_path.begin(), web_root_path.end()) > distance(path.begin(), path.end()) ||
				!equal(web_root_path.begin(), web_root_path.end(), path.begin()))
				throw invalid_argument("path must be within root path");
			if (boost::filesystem::is_directory(path))
				path /= "index.html";

			SimpleWeb::CaseInsensitiveMultimap header;

#ifdef HAVE_OPENSSL
			//    Uncomment the following lines to enable ETag
			//    {
			//      ifstream ifs(path.string(), ifstream::in | ios::binary);
			//      if(ifs) {
			//        auto hash = SimpleWeb::Crypto::to_hex_string(SimpleWeb::Crypto::md5(ifs));
			//        header.emplace("ETag", "\"" + hash + "\"");
			//        auto it = request->header.find("If-None-Match");
			//        if(it != request->header.end()) {
			//          if(!it->second.empty() && it->second.compare(1, hash.size(), hash) == 0) {
			//            response->write(SimpleWeb::StatusCode::redirection_not_modified, header);
			//            return;
			//          }
			//        }
			//      }
			//      else
			//        throw invalid_argument("could not read file");
			//    }
#endif

			auto ifs = make_shared<ifstream>();
			ifs->open(path.string(), ifstream::in | ios::binary | ios::ate);

			if (*ifs) {
				auto length = ifs->tellg();
				ifs->seekg(0, ios::beg);

				header.emplace("Content-Length", to_string(length));
				response->write(header);

				class FileServer {
				public:
					static void read_and_send(const shared_ptr<HttpServer::Response> &response, const shared_ptr<ifstream> &ifs) {
						// Read and send 128 KB at a time
						static vector<char> buffer(131072); // Safe when server is running on one thread
						streamsize read_length;
						if ((read_length = ifs->read(&buffer[0], static_cast<streamsize>(buffer.size())).gcount()) > 0) {
							response->write(&buffer[0], read_length);
							if (read_length == static_cast<streamsize>(buffer.size())) {
								response->send([response, ifs](const SimpleWeb::error_code &ec) {
									if (!ec)
										read_and_send(response, ifs);
									else
										cerr << "Connection interrupted" << endl;
								});
							}
						}
					}
				};
				FileServer::read_and_send(response, ifs);
			}
			else
				throw invalid_argument("could not read file");
		}
		catch (const exception &e) {
			response->write(SimpleWeb::StatusCode::client_error_bad_request, "Could not open path " + request->path + ": " + e.what());
		}
	};

	singleton_service->server.on_error = [](shared_ptr<HttpServer::Request> /*request*/, const SimpleWeb::error_code & /*ec*/) {
		// Handle errors here
		// Note that connection timeouts will also call this handle with ec set to SimpleWeb::errc::operation_canceled
	};

	singleton_service->server.start();
}

void stop_http_server() {
	singleton_service->server.stop();	
}
