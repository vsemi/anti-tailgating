#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <thread>
#include <mutex>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <pwd.h>
#include <algorithm>
#include <signal.h>

#include <arpa/inet.h>
#include <cerrno>
#include <ifaddrs.h>
#include <net/if.h>
#include <sysexits.h>
#include <sys/socket.h>

#include <sys/reboot.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Camera.hpp>
#include "people_counting.h"
#include "dgp.h"
#include "utils.h"

#include <sqlite3.h>

#include "MQTTAsync.h"
#include "lora.hpp"
#include "io.h"

#include "MJPGStreamer.h"
#include "tracking.h"

#include "http_service.h"

u_int32_t sensor_uid;

const char *sd_card = "/home/cat/data";
const char *comm_protocal = "relay";

char* action;

Camera* camera;
float *data_background;
float floor_height = 0.0;

volatile bool exit_requested = false;

unsigned int integrationTime0 = 1000;
unsigned int integrationTime1 = 50;

unsigned int amplitude0 = 20;
unsigned int amplitude1 = 20;

int hdr = 2;

bool is_stopped = false;
cv::Mat depth_bgr(60, 160, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Mat_<cv::Point3f> point3f_mat_(60, 160);

std::string title = "People Counting";

char const *LED_GREEN = "83";
char const *LED_RED   = "84";

char const *RELAY_0 = "34";
char const *RELAY_1 = "35";
char const *RELAY_2 = "36";
char const *RELAY_S = "37";

bool relay_high_effective = false;
bool wakeUp = false;

bool gpio_available = false;

std::string otg_ip_address = "10.42.0.1";

MJPGStreamer* streamer;

inline bool file_exists (const std::string& name) {
	if (name != "") {
		struct stat buffer;
		return (stat (name.c_str(), &buffer) == 0);
	}

	return false;
}

void relay_ON(char const *gpio_pin)
{
	if (relay_high_effective)
	{
		gpio_high(gpio_pin);
	} else 
	{
		gpio_low(gpio_pin);
	}
}

void relay_OFF(char const *gpio_pin)
{
	if (relay_high_effective)
	{
		gpio_low(gpio_pin);
	} else 
	{
		gpio_high(gpio_pin);
	}
}

bool has_ip_address(std::string ip)
{
    struct ifaddrs* ptr_ifaddrs = nullptr;
	//std::cout << "has ip address ...1" << std::endl;
    auto result = getifaddrs(&ptr_ifaddrs);
    if( result != 0 ){
        std::cout << "`getifaddrs()` failed: " << strerror(errno) << std::endl;

        return EX_OSERR;
    }
	//std::cout << "has ip address ...2" << std::endl;
    for(
        struct ifaddrs* ptr_entry = ptr_ifaddrs;
        ptr_entry != nullptr;
        ptr_entry = ptr_entry->ifa_next
    ){
        std::string ipaddress_human_readable_form;
        std::string netmask_human_readable_form;
		//std::cout << "has ip address ...2.0" << std::endl;
		if (ptr_entry == nullptr || ptr_entry->ifa_addr == nullptr) continue;
        std::string interface_name = std::string(ptr_entry->ifa_name);
		//std::cout << "has ip address ...2.0.0: " << ptr_entry->ifa_addr << std::endl;
		if (ptr_entry->ifa_addr)
		{
			sa_family_t address_family = ptr_entry->ifa_addr->sa_family;
			//std::cout << "has ip address ...2.1" << std::endl;
			if( address_family == AF_INET ){
				//std::cout << "has ip address ...2.1.0" << std::endl;
				if( ptr_entry != nullptr && ptr_entry->ifa_addr != nullptr ){
					//std::cout << "has ip address ...2.1.1" << std::endl;
					char buffer[INET_ADDRSTRLEN] = {0, };
					inet_ntop(
						address_family,
						&((struct sockaddr_in*)(ptr_entry->ifa_addr))->sin_addr,
						buffer,
						INET_ADDRSTRLEN
					);
					//std::cout << "has ip address ...2.1.2" << std::endl;
					ipaddress_human_readable_form = std::string(buffer);

					//std::cout << interface_name << ": IP address = " << ipaddress_human_readable_form << std::endl;

					if (ipaddress_human_readable_form == ip) return true;
				}
			}
		}
    }
	//std::cout << "has ip address ...3" << std::endl;
    freeifaddrs(ptr_ifaddrs);

    return false;
}

int detected_prev = -2;
int is_awake = false;

int ping_count = 0;
bool led_red_on = false;
int in = 0, out = 0;
void process(Camera* camera)
{
	ErrorNumber_e status;

	ToFImage tofImage(camera->getWidth(), camera->getHeight());
	camera->setIntegrationTimeGrayscale(0);

	std::chrono::steady_clock::time_point st_time0;
	std::chrono::steady_clock::time_point en_time0;

	std::chrono::steady_clock::time_point st_time;
	std::chrono::steady_clock::time_point en_time;

	double interval, frame_rate;
	st_time0 = std::chrono::steady_clock::now();
	st_time = std::chrono::steady_clock::now();
	int data_frame_id = 0;

	std::vector<SpatialObject> tracked_objects;

	while (! exit_requested)
	{
		try
		{
			status = camera->getDistance(tofImage);
			if (status != ERROR_NUMMBER_NO_ERROR)
			{
				std::cerr << "Error: " << status << std::endl;
				usleep(500000);
				continue;
			}

			if (strcmp(action, "train-detect") == 0 || strcmp(action, "train-view") == 0)
			{
				train(tofImage.data_3d_xyz_rgb, data_background);
				if (data_frame_id >= 5 && data_frame_id < 50 && data_frame_id % 5 == 0)
				{
					std::cout << "Training in progress: " << data_frame_id << "%, please wait ..." << std::endl;

					if (gpio_available) gpio_high(LED_RED);
				} else
				{
					if (gpio_available) gpio_low(LED_RED);
				}
				if (data_frame_id > 50)
				{
					std::string fn = std::string(sd_card) + "/model.bin";
					write(fn, data_background);
					for (int i = 0; i < 9600; i ++)
					{
						if (floor_height < data_background[i]) floor_height = data_background[i];
					}
					floor_height -= 0.1; 
					std::cout << "\nFloor: " << floor_height << std::endl;

					std::cout << "\nTraining completed, starting detect ..." << std::endl;

					if (strcmp(action, "train-detect") == 0)
					{
						action = (char*)"detect";
					} else if (strcmp(action, "train-view") == 0)
					{
						action = (char*)"view";
					}
					if (gpio_available) gpio_low(LED_RED);
				}
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

				pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tofImage.data_3d_xyz_rgb);
				std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tofImage.n_points);
				point_cloud_ptr->points.clear();
				point_cloud_ptr->points.insert(point_cloud_ptr->points.end(), pts.begin(), pts.end());
				point_cloud_ptr->resize(tofImage.n_points);
				point_cloud_ptr->width = tofImage.n_points;
				point_cloud_ptr->height = 1;
				point_cloud_ptr->is_dense = false;
				
				Cloud cloud;
				filterAndDownSample(point_cloud_ptr, cloud, data_background);

				std::vector<SpatialObject> parts_validated;
				int detected = detect(cloud, parts_validated);

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pre_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_detected(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_centers(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_debug(new pcl::PointCloud<pcl::PointXYZRGB>);

				detected = track(parts_validated, out, in, cloud_clustered, cloud_detected, cloud_centers, cloud_debug, tracked_objects, floor_height);
				if (tracked_objects.size() == 1)
				{
					if (tracked_objects[0].is_at_edge) detected = 0;
				} else if (tracked_objects.size() > 1)
				{
					bool one_in_center = false;
					for (int i = 0; i < tracked_objects.size(); i ++)
					{
						// at least one is in center area
						if (! tracked_objects[i].is_at_edge) {
							one_in_center = true;
							break;
						}
					}
					if (! one_in_center)
					{
						detected = 0;
					}
				}

				if (detected > 0 && ping_count%2 == 0)
				{
					ping_count = 0;

					if (! led_red_on)
					{
						if (gpio_available) gpio_high(LED_RED);
						led_red_on = true;
					} else
					{
						if (gpio_available) gpio_low(LED_RED);
						led_red_on = false;
					}
				} else if (detected == 0 && ping_count%50 == 0)
				{
					ping_count = 0;
					if (gpio_available) gpio_high(LED_RED);
					led_red_on = true;
				} else
				{
					if (gpio_available) gpio_low(LED_RED);
					led_red_on = false;
				}
				ping_count ++;

				int saturated_points = 0;
				for (int i = 0; i < tofImage.n_points; i ++)
				{
						if (tofImage.saturated_mask[i] > 0)
						{
								saturated_points ++;
						}
				}

				if (saturated_points > 500)
				{
						detected = -1;
				}

				if (strcmp(comm_protocal, "relay") == 0)
				{
					if (wakeUp)
					{
						//wake up check
						if (cloud.cloud->points.size() > 30)
						{
							if (! is_awake)
							{
								is_awake = true;
								if (gpio_available) relay_ON(RELAY_0);
								std::cout << "              relay -> wakeup ON" << std::endl;
							}
						} else
						{
							if (is_awake)
							{
								is_awake = false;
								if (gpio_available) relay_OFF(RELAY_0);
								std::cout << "              relay -> wakeup OFF" << std::endl;
							}
						}
					}
					if (detected_prev != detected)
					{
						if (detected_prev == 0) {
								if (! wakeUp)
								{
									if (gpio_available) relay_OFF(RELAY_0);
									std::cout << "              relay -> 0 OFF" << std::endl;
								}
						}
						if (detected_prev == 1) {
								if (gpio_available) relay_OFF(RELAY_1);
								std::cout << "              relay -> 1 OFF" << std::endl;
						}
						if (detected_prev >= 2) {
								if (gpio_available) relay_OFF(RELAY_2);
								std::cout << "              relay -> 2 OFF" << std::endl;
						}
						if (detected_prev == -1) {
								if (gpio_available) relay_OFF(RELAY_S);
								std::cout << "              relay -> S OFF" << std::endl;
						}

						if (detected == 0) {
							if (! wakeUp)
							{
								if (gpio_available) relay_ON(RELAY_0);
								std::cout << "              relay -> 0 ON" << std::endl;
							}
						}
						if (detected == 1) {
							if (gpio_available) relay_ON(RELAY_1);
							std::cout << "              relay -> 1 ON" << std::endl;
						}
						if (detected >= 2) {
							if (gpio_available) relay_ON(RELAY_2);
							std::cout << "              relay -> 2 ON" << std::endl;
						}
						if (detected == -1) {
							if (gpio_available) relay_ON(RELAY_S);
							std::cout << "              relay -> S ON" << std::endl;
						}

						detected_prev = detected;
					}
				}

				if (strcmp(action, "view") == 0)
				{
					depth_bgr = cv::Mat(tofImage.height, tofImage.width, CV_8UC3, tofImage.data_2d_bgr);

					cv::Mat depth_bgr_enlarge;
					if (hdr == 1)
					{
						cv::Rect roi(0, 0, 160, 30);
						cv::Mat depth_bgr_spatial = depth_bgr(roi);
						cv::resize(depth_bgr_spatial, depth_bgr_enlarge, cv::Size(1920, 720), cv::INTER_LINEAR);
					} else{
						cv::resize(depth_bgr, depth_bgr_enlarge, cv::Size(1920, 720), cv::INTER_LINEAR);
					}

					cv::Mat _blank(360, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
					cv::Mat depth_bgr_display;
					cv::vconcat(depth_bgr_enlarge, _blank, depth_bgr_display);

					cv::putText(
						depth_bgr_display, "Device ID: " + std::to_string(sensor_uid), cv::Point(20, 780),
						cv::FONT_HERSHEY_DUPLEX, 1.4, cv::Scalar(255,255,255), 1, cv::LINE_AA
					);

					cv::putText(
						depth_bgr_display, "Make sure part of the gate stub or door frame can be seen by ToF camera.", cv::Point(20, 840),
						cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(255,255,255), 1, cv::LINE_AA
					);

					cv::putText(
						depth_bgr_display, "For more info, visit http://www.vsemi.io", cv::Point(1020, 960),
						cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,255), 1, cv::LINE_AA
					);
					cv::putText(
						depth_bgr_display, "2023 Visionary Semiconductor Inc. All rights reserved", cv::Point(1020, 1000),
						cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,255), 1, cv::LINE_AA
					);

					if (detected == 0)
					{
						cv::putText(
							depth_bgr_display, "Persons Detected: " + std::to_string(detected), cv::Point(20, 960),
							cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,255), 1, cv::LINE_AA
						);
					} else if (detected == 1)
					{
						cv::putText(
							depth_bgr_display, "Persons Detected: " + std::to_string(detected), cv::Point(20, 960),
							cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 1, cv::LINE_AA
						);
					} else if (detected == 2 || detected == -1)
					{
						cv::putText(
							depth_bgr_display, "Persons Detected: " + std::to_string(detected), cv::Point(20, 960),
							cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 1, cv::LINE_AA
						);
					}

					streamer->write(depth_bgr_display);
				}
			}

			data_frame_id ++;
		}
		catch( ... )
		{
			std::cerr << "Unknown error in process ... " << std::endl;
		}
	}

	exit_requested = true;

	en_time0 = std::chrono::steady_clock::now();
	interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time0 - st_time0).count()) / 1000000.0;
	frame_rate = ((double) data_frame_id) / interval;
	std::cout << "Frames: " << data_frame_id << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;
}

void start()
{
	ErrorNumber_e status;

	status = camera->setOperationMode(MODE_BEAM_A);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set OperationMode failed." << std::endl;
	}

	status = camera->setModulationFrequency(MODULATION_FREQUENCY_20MHZ);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set ModulationFrequency failed." << std::endl;
	}

	status = camera->setModulationChannel(0, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set tModulationChannel failed." << std::endl;
	}

	camera->setAcquisitionMode(AUTO_REPEAT);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set AcquisitionMode failed." << std::endl;
	}

	status = camera->setOffset(0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set Offset failed." << std::endl;
	}

	status = camera->setIntegrationTime3d(0, integrationTime0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 0 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(1, integrationTime1);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 1 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(2, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 2 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(3, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 3 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(4, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 4 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(5, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 5 failed." << std::endl;
	}

	status = camera->setMinimalAmplitude(0, amplitude0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 0 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(1, amplitude1);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 1 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(2, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 2 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(3, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 3 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(4, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 4 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(5, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 5 failed." << std::endl;
	}

	camera->setRange(50, 5500);

	HDR_e hdr_mode = HDR_OFF;
	if (hdr == 2)
	{
		hdr_mode = HDR_TEMPORAL;
	} else if (hdr == 1)
	{
		hdr_mode = HDR_SPATIAL;
	}

	status = camera->setHdr(hdr_mode);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set HDR failed." << std::endl;
	}

	std::cout << "integrationTime0: " << integrationTime0 << std::endl;
	std::cout << "integrationTime1: " << integrationTime1 << std::endl;
	std::cout << "amplitude0:       " << amplitude0 << std::endl;
	std::cout << "amplitude1:       " << amplitude1 << std::endl;
	std::cout << "HDR:              " << hdr_mode << std::endl;
	std::cout << "\n" << std::endl;

	process(camera);

	delete camera;
}

void exit_handler(int s){
	std::cout << "\nExiting ... " << std::endl;
	exit_requested = true;
	fflush(stdout);
	usleep(1000000);
	signal(SIGINT, exit_handler);
}

int main(int argc, char** argv) {

	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	data_background = new float[9600];

	if (file_exists("/home/cat"))
	{
		gpio_available = true;
		otg_ip_address = "10.42.0.1";
	} else
	{
		sd_card = (char*) "/home/vsemi/data/peoplecount";
	}

	if (argc >= 2)
	{
		action = argv[1];
		std::cout << "Mode:            " << action << std::endl;
	} else {
		std::cout << "Usage: peoplecount [train-detect | train-view]"<< std::endl;
		return 1;
	}

	if (argc >= 3 && strcmp(argv[2], "wake") == 0)
	{
		wakeUp = true;
		std::cout << "                 wake up" << std::endl;
	}
	if (argc >= 3 && strcmp(argv[2], "high") == 0)
	{
		relay_high_effective = true;
		std::cout << "                 relay high effective" << std::endl;
	}
	if (argc >= 4 && strcmp(argv[3], "high") == 0)
	{
		relay_high_effective = true;
		std::cout << "                 relay high effective" << std::endl;
	}
	if (! relay_high_effective)
	{
		std::cout << "                 relay low effective" << std::endl;
	}

	bool otg_connected = has_ip_address(otg_ip_address);
	if (otg_connected) std::cout << "                 otg" << otg_connected << std::endl;

	usleep(2000000);

	if (gpio_available)
	{
		gpio_init(LED_RED);
		gpio_init(LED_GREEN);

		gpio_init(RELAY_0);
		gpio_init(RELAY_1);
		gpio_init(RELAY_2);
		gpio_init(RELAY_S);

		gpio_high(LED_RED);
		gpio_high(LED_GREEN);

		relay_OFF(RELAY_0);
		relay_OFF(RELAY_1);
		relay_OFF(RELAY_2);
		relay_OFF(RELAY_S);
	}

	if (! file_exists(sd_card))
	{
		std::cout << "No Data Storage found, please insert a Data Storage. " << std::endl;
		return 2;
	} else 
	{
		std::cout << "data Storage:    " << sd_card << std::endl;
	}
	usleep(2000000);
	
	if (gpio_available)
	{
		gpio_low(LED_RED);
	}
	usleep(1000000);

	bool tof_ok = false;
	std::cout << "" << std::endl;
	int attempts = 0;
	while ((! exit_requested) && (! tof_ok))
	{		
		int serial_n = attempts / 5;
		std::string sensor_port = "/dev/ttyACM" + std::to_string(serial_n);
		std::cout << "connect to ToF sensor at " << sensor_port << " ..." << std::endl;
		camera = Camera::usb_tof_camera_160(sensor_port);
		tof_ok = camera->open();

		if (! tof_ok)
		{
			std::cout << "    sensor at : " << sensor_port << " is not available." << std::endl;

			if (gpio_available)
			{
				gpio_high(LED_RED);
			}
			usleep(1000000);
			if (gpio_available)
			{
				gpio_low(LED_RED);
			}
			attempts ++;
			if (attempts >= 15) attempts = 0;
		} else
		{
			if (gpio_available)
			{
				gpio_high(LED_RED);
			}
			usleep(1000000);
			sensor_uid = camera->getID();
			std::cout << "sensor at : " << sensor_port << " connected." << std::endl;
		}
	}
	
	if (gpio_available)
	{
		gpio_low(LED_RED);
	}

	std::cout << "\n" << std::endl;
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "Device ID:         " << sensor_uid << std::endl;
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "\n" << std::endl;

	if (otg_connected)
	{
		action = (char*)"train-view";
	}

	if (strcmp(action, "train-view") == 0)
	{
		std::thread th_http(&start_http_server);
		th_http.detach();

		streamer = new MJPGStreamer();
		streamer->start(8800);

		if (gpio_available)
		{
			gpio_high(LED_RED);
		}

		integrationTime0 = 400;

		std::cout << "\nStarting view mode, point browser to http://10.42.0.1:8800 to view ToF distance iamge.\n" << std::endl;
	}

	start();

	if (strcmp(action, "view") == 0 || strcmp(action, "train-view") == 0)
	{
		stop_http_server();

		std::cout << "To stop streamer ..." << std::endl;
		streamer->stop();
		std::cout << "streamer stopped!" << std::endl;

		delete streamer;
		std::cout << "streamer deleted!" << std::endl;
	}

    if (gpio_available)
	{
		gpio_deinit(LED_RED);
        gpio_deinit(LED_GREEN);

		gpio_deinit(RELAY_0);
		gpio_deinit(RELAY_1);
		gpio_deinit(RELAY_2);
		gpio_deinit(RELAY_S);
	}

	return 0;
}
