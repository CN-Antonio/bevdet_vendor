#include "test.h"

// 添加从opencv的Mat转换到std::vector<char>的函数 读取图像 cv2data 
int cvToArr(cv::Mat img, std::vector<char> &raw_data)
{
    if (img.empty())
    {
        std::cerr << "image is empty. " << std::endl;
        return EXIT_FAILURE;
    }
    
    std::vector<u_char> raw_data_;
    cv::imencode(".jpg", img, raw_data_);
    raw_data = std::vector<char>(raw_data_.begin(), raw_data_.end());
    return EXIT_SUCCESS;
}

int cvImgToArr(std::vector<cv::Mat> &imgs, std::vector<std::vector<char>> &imgs_data)
{
    imgs_data.resize(imgs.size());

    for(size_t i = 0; i < imgs_data.size(); i++)
    {   
        if(cvToArr(imgs[i], imgs_data[i]))
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

// test

namespace bevdet::bevdet_ros
{
void BEVDet_Node::timer_callback()
{
    if(timer_count_>999) timer_count_=0;

    auto message = std_msgs::msg::String();
    message.data = "Hello_World: " + std::to_string(timer_count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    timer_count_++;
}
}  // namespace bevdet::bevdet_ros