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

bool wakeUp = false;

bool gpio_available = false;

std::string otg_ip_address = "10.42.0.1";//"10.10.31.191";

MJPGStreamer* streamer;

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> queque_record;
std::mutex mutex_queque_record;

inline bool file_exists (const std::string& name) {
	if (name != "") {
		struct stat buffer;
		return (stat (name.c_str(), &buffer) == 0);
	}

	return false;
}

bool has_ip_address(std::string ip)
{
    struct ifaddrs* ptr_ifaddrs = nullptr;

    auto result = getifaddrs(&ptr_ifaddrs);
    if( result != 0 ){
        std::cout << "`getifaddrs()` failed: " << strerror(errno) << std::endl;

        return EX_OSERR;
    }

    for(
        struct ifaddrs* ptr_entry = ptr_ifaddrs;
        ptr_entry != nullptr;
        ptr_entry = ptr_entry->ifa_next
    ){
        std::string ipaddress_human_readable_form;
        std::string netmask_human_readable_form;

        std::string interface_name = std::string(ptr_entry->ifa_name);
        sa_family_t address_family = ptr_entry->ifa_addr->sa_family;
        if( address_family == AF_INET ){

            if( ptr_entry->ifa_addr != nullptr ){
                char buffer[INET_ADDRSTRLEN] = {0, };
                inet_ntop(
                    address_family,
                    &((struct sockaddr_in*)(ptr_entry->ifa_addr))->sin_addr,
                    buffer,
                    INET_ADDRSTRLEN
                );

                ipaddress_human_readable_form = std::string(buffer);

				std::cout << interface_name << ": IP address = " << ipaddress_human_readable_form << std::endl;

				if (ipaddress_human_readable_form == ip) return true;
            }
        }
    }

    freeifaddrs(ptr_ifaddrs);

    return false;
}

int empty_frames_interval = 0;
bool pause_process = false;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_test_0(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_test_1(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_test_2(new pcl::PointCloud<pcl::PointXYZRGB>);
int test_index = 0;

int detected_prev = -2;
int is_awake = false;

int ping_count = 0;
bool led_green_on = false;
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

			if (strcmp(action, "train") == 0 || strcmp(action, "train-detect") == 0 || strcmp(action, "train-record") == 0 || strcmp(action, "train-view") == 0)
			{
				train(tofImage.data_3d_xyz_rgb, data_background);
				if (data_frame_id >= 5 && data_frame_id < 50 && data_frame_id % 5 == 0)
				{
					std::cout << "Training in progress: " << data_frame_id << "%, please wait ..." << std::endl;

					if (gpio_available) gpio_high(LED_GREEN);
				} else
				{
					if (gpio_available) gpio_low(LED_GREEN);
				}
				if (data_frame_id > 50)
				{
					std::string fn = std::string(sd_card) + "/model.bin";
					write(fn, data_background);
					for (int i = 0; i < 9600; i ++)
					{
						if (floor_height < data_background[i]) floor_height = data_background[i];
					}
					floor_height -= 0.1; //offset
					std::cout << "   floor_height: " << floor_height << std::endl;

					std::cout << "\nTraining completed, starting detect ..." << std::endl;

					if (strcmp(action, "train") == 0)
					{
						break;
					} else if (strcmp(action, "train-detect") == 0)
					{
						action = (char*)"detect";
					} else if (strcmp(action, "train-record") == 0)
					{
						action = (char*)"record";
					} else if (strcmp(action, "train-view") == 0)
					{
						action = (char*)"view";
					}
					if (gpio_available) gpio_low(LED_GREEN);
				}
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

				if (
					strcmp(action, "detect") == 0 ||
					strcmp(action, "record") == 0 ||
					strcmp(action, "view") == 0
				)
				{
					pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tofImage.data_3d_xyz_rgb);
					std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tofImage.n_points);
					point_cloud_ptr->points.clear();
					point_cloud_ptr->points.insert(point_cloud_ptr->points.end(), pts.begin(), pts.end());
					point_cloud_ptr->resize(tofImage.n_points);
					point_cloud_ptr->width = tofImage.n_points;
					point_cloud_ptr->height = 1;
					point_cloud_ptr->is_dense = false;
				}
				else if (strcmp(action, "test") == 0)
				{
					if (test_index < 10)
					{
						*point_cloud_ptr = *cloud_test_0;
					} else if (test_index >= 10 && test_index < 20)
					{
						*point_cloud_ptr = *cloud_test_1;
					} else if (test_index >= 20 && test_index < 30)
					{
						*point_cloud_ptr = *cloud_test_2;
					}
					test_index ++;
					if (test_index >= 30)
					{
						test_index = 0;
					}
				}

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
					//std::cout << "   Track ... " << std::endl;

					if (! led_green_on)
					{
						if (gpio_available) gpio_high(LED_GREEN);
						led_green_on = true;
					} else
					{
						if (gpio_available) gpio_low(LED_GREEN);
						led_green_on = false;
					}
				} else if (detected == 0 && ping_count%50 == 0)
				{
					ping_count = 0;
					if (gpio_available) gpio_high(LED_GREEN);
					led_green_on = true;
				} else
				{
					if (gpio_available) gpio_low(LED_GREEN);
					led_green_on = false;
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
								if (gpio_available) gpio_low(RELAY_0);
								std::cout << "======> power relay -> wakeup ON" << std::endl;
							}
						} else
						{
							if (is_awake)
							{
								is_awake = false;
								if (gpio_available) gpio_high(RELAY_0);
								std::cout << "======> power relay -> wakeup OFF" << std::endl;
							}
						}
					}
					if (detected_prev != detected)
					{
							//gpio_high //gpio_low
						if (detected_prev == 0) {
								if (! wakeUp)
								{
									if (gpio_available) gpio_high(RELAY_0);
									std::cout << "======> power relay -> 0 OFF" << std::endl;
								}
						}
						if (detected_prev == 1) {
								if (gpio_available) gpio_high(RELAY_1);
								std::cout << "======> power relay -> 1 OFF" << std::endl;
						}
						if (detected_prev >= 2) {
								if (gpio_available) gpio_high(RELAY_2);
								std::cout << "======> power relay -> 2 OFF" << std::endl;
						}
						if (detected_prev == -1) {
								if (gpio_available) gpio_high(RELAY_S);
								std::cout << "======> power relay -> S OFF" << std::endl;
						}

						if (detected == 0) {
							if (! wakeUp)
							{
								if (gpio_available) gpio_low(RELAY_0);
								std::cout << "======> power relay -> 0 ON" << std::endl;
							}
						}
						if (detected == 1) {
							if (gpio_available) gpio_low(RELAY_1);
							std::cout << "======> power relay -> 1 ON" << std::endl;
						}
						if (detected >= 2) {
							if (gpio_available) gpio_low(RELAY_2);
							std::cout << "======> power relay -> 2 ON" << std::endl;
						}
						if (detected == -1) {
							if (gpio_available) gpio_low(RELAY_S);
							std::cout << "======> power relay -> S ON" << std::endl;
						}

						detected_prev = detected;
					}
				}

				if (strcmp(action, "record") == 0)
				{
					std::cout << "======> cloud.cloud->points: " << cloud.cloud->points.size() << " empty_frames_interval: " << empty_frames_interval << std::endl;
					std::unique_lock<std::mutex> lock_queque_record(mutex_queque_record);
					if (cloud.cloud->points.size() > 30)
					{
						queque_record.push_back(point_cloud_ptr);
						empty_frames_interval = 0;
					} else
					{
						if (empty_frames_interval < 5)
						{
							queque_record.push_back(point_cloud_ptr);
							empty_frames_interval ++;
						}
					}
					lock_queque_record.unlock();
				}

				if (strcmp(action, "view") == 0 || strcmp(action, "test") == 0)
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

					if (strcmp(action, "test") == 0)
					{
						cv::imshow("Anti-Tailgating", depth_bgr_display);
						if (cv::waitKey(1) == 27)
						{
							exit_requested = true;
						}
					}

					if (strcmp(action, "view") == 0)
					{
						streamer->write(depth_bgr_display);
					}
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

int i_frame_id = 0;
int i_frame_total = 0;
bool refresh_folder_path(std::string &f_path)
{
	char date_str[9];
	std::time_t t = std::time(NULL);
	std::strftime(date_str, sizeof(date_str), "%Y%m%d", std::localtime(&t));
	f_path = std::string(sd_card) + "/" + std::string(date_str);

	if (!file_exists(f_path))
	{
		int status = mkdir(f_path.c_str(),0777);
		if (status == 0)
		{
			std::cerr << "Folder " << f_path << " created, recording started ..." << std::endl;
			i_frame_id = 0;
		} else
		{
			std::cerr << "Failed to create folder in " << sd_card << ", recording skipped!" << std::endl;
			return false;
		}
	}

	return true;
}

void record_data()
{
	std::string f_path = "";
	bool file_system_ok = refresh_folder_path(f_path);
	if (! file_system_ok)
	{
		exit_requested = true;
	}

	while (! exit_requested)
	{
		try
		{

			if (queque_record.size() > 0)
			{
				file_system_ok = refresh_folder_path(f_path);
				if (! file_system_ok)
				{
					exit_requested = true;
				}

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw = queque_record[0];
				queque_record.erase (queque_record.begin());

				std::string fn = f_path + "/" + std::to_string(i_frame_id) + ".pcd";

				//std::cerr << "Saving data into fn: " << fn << std::endl;
				pcl::io::savePCDFileBinary(fn.c_str(), *cloud_raw);

				if (i_frame_total > 50000) {
					exit_requested = true;
					std::cerr << "Maximum number of frames reached: " << i_frame_total << std::endl;
					std::cerr << "Please clean data storage and restart recording again! " << std::endl;
				}

				i_frame_id ++;
				i_frame_total ++;
			} else{
				usleep(1);
			}
		} catch (...)
		{
			std::cerr << "Unknown error in record_data ... " << std::endl;
		}
	}
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
	std::cout << "amplitude0:        " << amplitude0 << std::endl;
	std::cout << "amplitude1:        " << amplitude1 << std::endl;
	std::cout << "HDR:              " << hdr_mode << std::endl;
	std::cout << "\n" << std::endl;

	if (strcmp(action, "record") == 0 || strcmp(action, "train-record") == 0)
	{
		std::thread th_record_data(&record_data);
		th_record_data.detach();
	}

	process(camera);

	delete camera;
}

void exit_handler(int s){
	std::cout << "\nExiting ... " << std::endl;
	exit_requested = true;
	fflush(stdout);
	usleep(2000000);
	signal(SIGINT, exit_handler);
	//exit(0);
}

int main(int argc, char** argv) {

	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	data_background = new float[9600];

	std::string test_data_path = "/home/vsemi/dev/anti_tailgating/test/";
	if (file_exists("/home/cat"))
	{
		test_data_path = "/home/cat/anti_tailgating/test/";
		gpio_available = true;
		otg_ip_address = "10.42.0.1";
	} else
	{
		sd_card = (char*) "/home/vsemi/data/peoplecount/test";
	}

	if (argc >= 2)
	{
		action = argv[1];
		std::cout << "Mode:            " << action << std::endl;
	} else {
		std::cout << "Usage: peoplecount [train-detect | train-record | train-view | test]"<< std::endl;
		return 1;
	}

	if (argc >= 3 && strcmp(argv[2], "wake") == 0)
	{
		wakeUp = true;
	}

	if (gpio_available)
	{
		gpio_init(LED_RED);
		gpio_init(LED_GREEN);

		gpio_init(RELAY_0);
		gpio_init(RELAY_1);
		gpio_init(RELAY_2);
		gpio_init(RELAY_S);

		gpio_low(LED_RED); // low on
		gpio_high(LED_GREEN); // high on

		gpio_high(RELAY_0);
		gpio_high(RELAY_1);
		gpio_high(RELAY_2);
		gpio_high(RELAY_S);
	}

	if (strcmp(action, "test") == 0)
	{
		read(test_data_path + "model.bin", data_background);
		for (int i = 0; i < 9600; i ++)
		{
			if (floor_height < data_background[i]) floor_height = data_background[i];
		}
		floor_height -= 0.1; //offset
		std::cout << "   floor_height: " << floor_height << std::endl;
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (test_data_path + "0.pcd", *cloud_test_0);
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (test_data_path + "1.pcd", *cloud_test_1);
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (test_data_path + "2.pcd", *cloud_test_2);
	}

	if (! file_exists(sd_card))
	{
		std::cout << "No Data Storage found, please insert a Data Storage. " << std::endl;
		return 2;
	}

	usleep(1000000);
	if (gpio_available)
	{
		gpio_high(LED_RED);
		gpio_low(LED_GREEN);
	}

	bool tof_ok = false;
	std::cout << "Connect to ToF sensor ... " << std::endl;
	usleep(1000000);
	while ((! exit_requested) && (! tof_ok))
	{
		if (gpio_available)
		{
			gpio_low(LED_RED);
			usleep(3000000);
		}
		camera = Camera::usb_tof_camera_160("/dev/ttyACM0");
		tof_ok = camera->open();

		if (! tof_ok)
		{
			usleep(3000000);
			if (gpio_available)
			{
				gpio_high(LED_RED);
			}
			std::cout << "Opening ToF sensor ..." << std::endl;
		} else
		{
			sensor_uid = camera->getID();
		}
	}
	usleep(1000000);
	if (gpio_available)
	{
		gpio_low(LED_RED);
	}

	std::cout << "\n" << std::endl;
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "Device ID:         " << sensor_uid << std::endl;
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "\n" << std::endl;

	bool otg_connected = has_ip_address(otg_ip_address);
	std::cout << "otg connected ..." << otg_connected << std::endl;

	if (otg_connected)
	{
		action = (char*)"train-view";
	}

	if (strcmp(action, "view") == 0 || strcmp(action, "train-view") == 0)
	{
		std::thread th_http(&start_http_server);
		th_http.detach();

		streamer = new MJPGStreamer();
		streamer->start(8800);

		if (gpio_available)
		{
			gpio_low(LED_RED);
			gpio_high(LED_GREEN);
		}

		integrationTime0 = 400;

		std::cout << "\nStarting view mode, point browser to http://10.42.0.1:8800 to view ToF distance iamge.\n" << std::endl;
	}
	else if (strcmp(action, "test") == 0)
	{
		if (gpio_available)
		{
			gpio_low(LED_RED);
			gpio_high(LED_GREEN);
		}
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
