/**
 * @file util.hpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief Utilクラスのヘッダーファイル
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include "common_header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <common_msgs/BoxPosition.h>
#include <common_msgs/ObjectInfo.h>


typedef std::vector<int> ArrayInt;
typedef std::vector<double> ArrayDouble;
struct ImageSize
{
    int width;
    int height;
};

struct YoloFormat
{
  float x;
  float y;
  float w;
  float h;
  std::string tf_name;
  std::string object_class_name;
};

class Util
{
public:
    Util();
    template <typename T>
    static std::vector<T> concatenate_array(std::vector<T> array_original, std::vector<T> array_add)
    {
        array_original.insert(array_original.end(), array_add.begin(), array_add.end());
        return array_original;
    }
    static int calcurate_round_up(double);
    static void mkdir(std::string);
    static std::string join(std::string, std::string);
    /**
     * @brief 変数を格納してその変数がどの型なのかを示します。
     *
     * @tparam 探りたい変数
     * @return 型
     */
    template <typename T>
    static std::string type(T para)
    {
        return Util::get_name_by_typeinfo(typeid(para));
    }

    static std::string get_time_str();
    static double distance(double[], double[]);
    static double distance(double x1, double y1, double x2, double y2);
    

    /**
     * @brief 標準出力に変数の内容を表示します。主に手動のデバックで用います。
     *
     * @tparam T
     * @param name: 表示されている箇所がどこなのか特定するために名前を表示します。
     * @param variable: 中身を表示する変数
     */
    template <typename T>
    static void message_show(std::string name, T variable)
    {
        try
        {
            std::cout << name << ": " << variable << std::endl;
        }
        catch (...)
        {
            ROS_ERROR("メッセージが表示できない変数です");
        }
    }

    /**
     * @brief クライアントがリクエストを送信します。
     *
     * @tparam T
     * @param client ros::ServiceClientを格納
     * @param msg サービスメッセージを格納
     * @param service_name
     */
    template <typename T>
    static void client_request(ros::ServiceClient client, T &msg, std::string service_name)
    {
        ros::service::waitForService(service_name);
        if (client.call(msg))
        {
            // ROS_INFO_STREAM("client request: " << service_name);
        }
        else
        {
            ROS_ERROR_STREAM(service_name << " service error!");
            return;
        }
    }

    template <typename T>
    static bool is_same_element_exist(std::vector<T> list, int current_index)
    {
        for (int i = 0; i < current_index; i++) {
            if (list[i] == list[current_index]) {
                return true;
            }
        }
        return false;
    }
    static tf::StampedTransform make_stamped_trans(geometry_msgs::Transform);
    int random_int(int, int);
    float random_float(float, float);
    static ImageSize get_image_size(cv::Mat);
    XmlRpc::XmlRpcValue param_list;
    static void box_position_show(common_msgs::BoxPosition, std::string);
    static void yolo_format_show(YoloFormat, std::string);
    static std::vector<common_msgs::ObjectInfo> delete_empty_object_info(std::vector<common_msgs::ObjectInfo>);

private:
    std::random_device rd_;
    std::default_random_engine eng_;
    static std::string get_name_by_typeinfo(std::type_info const &);
};
