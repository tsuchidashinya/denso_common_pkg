/**
 * @file util_base.hpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief UtilBaseクラスのヘッダーファイル
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include "common_header.hpp"


class UtilBase
{
public:
    UtilBase();
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
        return UtilBase::get_name_by_typeinfo(typeid(para));
    }

    static std::string get_time_str();
    static double distance(double[], double[]);
    static geometry_msgs::Transform geo_trans_make(double, double, double, tf2::Quaternion);

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
            ;
        }
        else
        {
            ROS_ERROR_STREAM(service_name << " service error!");
            return;
        }
    }

    
    tf::StampedTransform make_stamped_trans(geometry_msgs::Transform);
    int random_int(int, int);
    float random_float(float, float);

    XmlRpc::XmlRpcValue param_list;

private:
    
    std::random_device rd_;
    std::default_random_engine eng_;

    static std::string get_name_by_typeinfo(std::type_info const &);
};
