
#include <mutex>
#include <thread>
#include <condition_variable>

#ifdef _WIN32
#include <winsock.h>
#include <windows.h>
#include <time.h>
#define PORT        unsigned long
#define ADDRPOINTER   int*
#else       /* ! win32 */
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define PORT        unsigned short
#define SOCKET    int
#define HOSTENT  struct hostent
#define SOCKADDR    struct sockaddr
#define SOCKADDR_IN  struct sockaddr_in
#define ADDRPOINTER  unsigned int*
#define INVALID_SOCKET -1
#define SOCKET_ERROR   -1
#endif /* _WIN32 */

#define TIMEOUT_M       200000
#define NUM_CONNECTIONS 100

#ifdef _WIN32
#pragma comment(lib,"WS2_32")
#endif

#include "opencv2/opencv.hpp"

#include "StreamBuffer.h"

class MJPGStreamer{
	class Client
	{
	private:
		int _id;
		MJPGStreamer* _streamer;
		bool _alive;
		int buf_len = 0;
		std::vector<uchar> outbuf;

		std::mutex mutex_client_writer;
		std::condition_variable condition_variable_client_writer;
		std::thread m_thread_client_writer;

		void run();
	public:
		Client(int id);
		~Client();
		void setStreamer(MJPGStreamer* streamer);
		void start();
		void stop();
		void feed_data(std::vector<uchar> data, int len);
		bool isLive();
	};
	SOCKET sock;
	fd_set master;
	int timeout; // master sock timeout, shutdown after timeout millis.
	int quality; // jpeg compression [1..100]
	int port;

	bool m_stopped = false;

	std::map<int, std::shared_ptr<Client>> m_clients;
	std::mutex mutex_clients;
	std::thread thread_listener;

	std::mutex mutex_writer;
	std::condition_variable condition_variable_writer;
	std::thread m_thread_writer;

	std::mutex mutex_dead_conn_collector;
	std::condition_variable condition_dead_conn_collector;
	std::thread m_thread_dead_conn_collector;
	std::vector<int> dead_connections;

	cv::Mat lastFrame;

	int _write(int sock, char *s, int len);

public:

	MJPGStreamer();

	~MJPGStreamer();

	void setQuality(float quality);

	bool open();

	bool isOpened();

	void start(int p);

	void stop();

	void listen_new_client();

	void write(const cv::Mat & frame);

	void clientWrite();

	void clean_dead_connections();
};
