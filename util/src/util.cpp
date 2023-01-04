/**
 * @file util.cpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief  Utilクラスのソースファイル
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <util/util.hpp>

Util::Util()
    : 
      rd_(),
      eng_(rd_())
{
}

float Util::probability()
{
  return random_float(0, 1);
}

void Util::write_b_box_label(std::vector<common_msgs::BoxPosition> boxes, std::string save_file_path)
{
    std::ofstream file(save_file_path);
    for (int i = 0; i < boxes.size(); i++) {
        file << boxes[i].object_name;
        file << " " << boxes[i].x_one << " " << boxes[i].y_one;
        file << " " << boxes[i].x_two << " " << boxes[i].y_two;
        file << std::endl;
    }
}

int Util::find_tfname_from_cloudlist(std::vector<common_msgs::CloudData> cloud_list, std::string tf_name) {
  int index = -1;
  for (int i = 0; i < cloud_list.size(); i++) {
    if (cloud_list[i].tf_name == tf_name) {
      index = i;
    }
  }
  return index;
}

/**
 * @brief 小数点以下に少しでも値があれば+1した整数値を渡す関数
 *
 * @param num
 * @return int
 */
int Util::calcurate_round_up(double num)
{
  if (num - int(num))
  {
    return int(num) + 1;
  }
  else
  {
    return int(num);
  }
}




/**
 * @brief ディレクトリを生成
 *
 * @param path
 */
void Util::mkdir(std::string path)
{
  if (!std::filesystem::exists(path))
  {
    std::filesystem::create_directories(path);
  }
}

/**
 * @brief pythonのos.path.joinと同じ機能を持つ、パスの結合関数
 *
 * @param str_front 結合されるパス
 * @param str_add 結合するパス
 * @return std::string 結合後のパス
 */
std::string Util::join(std::string str_front, std::string str_add)
{
  if (str_front[str_front.size() - 1] == '/')
  {
    str_front = str_front.substr(0, str_front.size() - 1);
  }
  if (str_add[str_add.size() - 1] == '/')
  {
    str_add = str_add.substr(0, str_add.size() - 1);
  }
  if (str_add[0] == '/')
  {
    str_add = str_add.substr(1, str_add.size());
  }
  std::string output;
  output = str_front + "/" + str_add;
  return output;
}

/**
 * @brief 現在の時刻などをパスに表示するための関数
 *
 * @return std::string 月_日_時間_分_秒
 */
std::string Util::get_time_str()
{
  time_t t_ = time(nullptr);
  const tm *localtime_ = localtime(&t_);
  std::string output = std::to_string(localtime_->tm_mon + 1) + "_" + std::to_string(localtime_->tm_mday) + "_" + std::to_string(localtime_->tm_hour) + "_" + std::to_string(localtime_->tm_min) + "_" + std::to_string(localtime_->tm_sec);
  return output;
}


/*
1: x1
2: y1
3: x2
4: y2
*/
double Util::distance(double x1, double y1, double x2, double y2)
{
    return sqrt(abs(x1 - x2)*abs(x1 - x2) + abs(y1 - y2) * abs(y1 - y2));
}




/**
 * @brief int型のランダムな数値を生成する
 *
 * @param min 生成する数値の下限
 * @param max 生成する数値の上限
 * @return int
 */
int Util::random_int(int min, int max)
{
  std::uniform_int_distribution<> distr(min, max);
  return distr(eng_);
}

/**
 * @brief float型のランダムな数値を生成する
 *
 * @param min 生成する数値の下限
 * @param max 生成する数値の上限
 * @return float
 */
float Util::random_float(float min, float max)
{
  std::uniform_real_distribution<> distr(min, max);
  return distr(eng_);
}

std::string Util::get_name_by_typeinfo(std::type_info const &iTypeInfo)
{
  char *aName;
  int status = 0;
  aName = abi::__cxa_demangle(iTypeInfo.name(), 0, 0, &status);
  std::string ret(aName);
  std::free(aName);
  return ret;
}

ImageSize Util::get_image_size(cv::Mat img)
{
    ImageSize outdata;
    outdata.height = img.rows;
    outdata.width = img.cols;
    return outdata;
}

void Util::box_position_show(common_msgs::BoxPosition box_pos, std::string title)
{
  ROS_INFO_STREAM(title << ": " << box_pos.x_one << " " << box_pos.x_two << " " << box_pos.y_one << " " << box_pos.y_two);
}

void Util::yolo_format_show(YoloFormat yolo_data, std::string title)
{
  ROS_INFO_STREAM(title << ": " << yolo_data.x << " " << yolo_data.y << " " << yolo_data.w << " " << yolo_data.h);
}

std::vector<common_msgs::ObjectInfo> Util::delete_empty_object_info(std::vector<common_msgs::ObjectInfo> object_info)
{
  std::vector<common_msgs::ObjectInfo> outdata;
  for (int i = 0; i < object_info.size(); i++) {
    if (object_info[i].object_name.size() == 0 || object_info[i].tf_name.size() == 0) {
      ;
    }
    else {
      outdata.push_back(object_info[i]);
    }
  }
  return outdata;
}