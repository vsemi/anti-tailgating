#pragma once
#include <mutex>
#include <opencv2/core.hpp>

class StreamBuffer {
private:
	std::recursive_mutex m_mutex_frame;
	cv::Mat m_current_frame;
public:
	StreamBuffer();
	void update_current_frame(cv::Mat* m);
	cv::Mat get_current_frame();
};