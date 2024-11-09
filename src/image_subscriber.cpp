// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// Cv bridge to convert from ROS image to OpenCV image
#include <cv_bridge/cv_bridge.h>

// Standard library includes
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

// For timing the DMA transfer
#include <time.h>
#include <chrono>

// For DMA transfer
#include "axi_dma_controller.h"
#include "reserved_mem.hpp"

// Defines used for the AXI DMA Controller
#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define LENGTH 0x007fffff // Length in bytes
#define P_START 0x70000000
#define P_OFFSET 0
#define UIO_DMA_N 0

// For timekeeping;
std::chrono::_V2::system_clock::time_point t1;

void start_timer()
{
	t1 = std::chrono::high_resolution_clock::now();
	// std::cout << "Start timer" << std::endl;
}
double stop_timer()
{
	auto t2 = std::chrono::high_resolution_clock::now();
	auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
	std::chrono::duration<double, std::milli> ms_double = t2 - t1;
	// std::cout << "Duration: " << ms_double.count() << "ms [" << (float)LENGTH / 1000000. << "MB]" << std::endl;
	return ms_double.count();
}

void print_mem(void *virtual_address, int byte_count)
{
	char *data_ptr = (char *)virtual_address;

	for (int i = 0; i < byte_count; i++)
	{
		printf("%02X", data_ptr[i]);

		// print a space every 4 bytes (0 indexed)
		if (i % 4 == 3)
		{
			printf(" ");
		}
	}

	printf("\n");
}


// Class to handle the image subscription
class ImageSubscriber : public rclcpp::Node
{
	public:
		ImageSubscriber() : Node("image_subscriber") {
			RCLCPP_INFO(this->get_logger(), "Initializing ImageSubscriber node");

			RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
					"/image_raw",
					10,
					std::bind(&ImageSubscriber::onImageMsg, this, std::placeholders::_1)
			);

			camera_publisher = this->create_publisher<sensor_msgs::msg::Image>("/image_gray", 10);

		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher;

		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received image!");

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			cv::Mat cam_img = cv_ptr->image;

			cv::Mat gray_img(cam_img.rows, cam_img.cols, CV_8UC1);
			cv::cvtColor(cam_img, gray_img, cv::COLOR_YUV2GRAY_YUYV);

			uint32_t data_stream[640*480];
			for (int row = 0; row < 480; row++)
			{
				for (int col = 0; col < 640; col++)
				{
					data_stream[row*640 + col] = (uint32_t)gray_img.at<uint8_t>(row, col);
				}
			}

			//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			// AXI DMA Transfer
			double total_t = 0;
			double tmp = 0;

			// Objects for handling the reserved memory and DMA controller
			printf("Running DMA transfer with specified memory.\n");
			Reserved_Mem pmem;
			AXIDMAController dma(UIO_DMA_N, 0x10000);
			uint32_t *u_buff = (uint32_t *)malloc(LENGTH);
			if (u_buff == NULL)
			{
				printf("could not allocate user buffer\n");
				return -1;
			}

			for (int i = 0; i < LENGTH / sizeof(uint32_t); i++)
				u_buff[i] = i * 2;
			printf("User memory reserved and filled\n");
			
			// Transfer the data to the reserved memory
			tmp = 0;
			start_timer();
			pmem.transfer(u_buff, P_OFFSET, LENGTH);
			total_t += stop_timer();
			std::cout << "Data transfered to reserved memory: " << total_t << "ms [" << (float)LENGTH / 1000000. << "MB]" << std::endl;
			
			// Reset the DMA
			start_timer();
			printf("Reset the DMA.\n");
			dma.MM2SReset();

			// Get the status of the DMA
			DMAStatus mm2s_status = dma.MM2SGetStatus();

			// Halt the DMA
			printf("Halt the DMA.\n");
			dma.MM2SHalt();

			// Enable DMA interrupts
			printf("Enable all interrupts.\n");
			dma.MM2SInterruptEnable();

			// Set the source address of the data
			printf("Writing source address of the data from MM2S in DDR...\n");
			dma.MM2SSetSourceAddress(P_START + P_OFFSET);

			// Start data transfer
			dma.MM2SStart();

			// Set the length of the data, cant be larger then 2^23
			dma.MM2SSetLength(LENGTH);

			tmp = stop_timer();
			total_t+=tmp;
			std::cout << "\nDMA setup done, transfer begun: " << tmp << "ms [" << (float)LENGTH / 1000000. << "MB]\n" << std::endl;

			start_timer();
			printf("...Waiting for MM2S synchronization...\n");

			while (!dma.MM2SIsSynced()){}

			tmp = stop_timer();
			total_t += tmp;
			std::cout << "\nData transfered to transfered by DMA: " << tmp << "ms [" << (float)LENGTH / 1000000. << "MB]\n" << std::endl;
			//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

			std::cout << "pixel(0,0): " << (int)data_stream[0] << std::endl;

			sensor_msgs::msg::Image::SharedPtr message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_img).toImageMsg();
      		camera_publisher->publish(*message.get());

			RCLCPP_INFO(this->get_logger(), "Successfully loaded image");
		}

};

int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<ImageSubscriber>());

	rclcpp::shutdown();
	return 0;
}
