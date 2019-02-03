/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_COMMON_IO_H
#define ROBORTS_COMMON_IO_H
#include <iomanip>
#include <iostream>
#include <string>
#include <stdint.h>

#include <fcntl.h>
#include <unistd.h>

#include <google/protobuf/message.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <ros/package.h>
#include <ros/ros.h>


namespace roborts_common{
const int kProtoReadBytesLimit = INT_MAX;  // Max size of 2 GB minus 1 byte.

template<class T>   //用函数模板避免了函数重载的工作量
inline bool ReadProtoFromTextFile(const char *file_name, T *proto) {    //用内联函数减少调用消耗的跳转时间 明显是经常调用的函数
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  using google::protobuf::io::CodedInputStream;
  using google::protobuf::io::ZeroCopyOutputStream;
  using google::protobuf::io::CodedOutputStream;
  using google::protobuf::Message;                          //用了一堆google的命名空间？ 用来进行交互

  std::string full_path = /*ros::package::getPath("roborts") +*/ std::string(file_name);    //将传入的路径另存
  ROS_INFO("Load prototxt: %s", full_path.c_str());   //终端提示

  int fd = open(full_path.c_str(), O_RDONLY); //以只读的方式打开文件
  if (fd == -1) {
    ROS_ERROR("File not found: %s", full_path.c_str());   //打开失败 报错跳出
    return false;
  }
  FileInputStream *input = new FileInputStream(fd);     //创造输入流
  bool success = google::protobuf::TextFormat::Parse(input, proto);  //将文件内容赋值给proto 同时返回成功与否的指示变量
  delete input;   //删除变量
  close(fd);  //关闭文件 方便别的程序调用
  return success;
}

template<class T>
inline bool ReadProtoFromTextFile(const std::string &file_name, T *proto) {
  return ReadProtoFromTextFile(file_name.c_str(), proto);
}

template<class T>
inline bool ReadProtoFromBinaryFile(const char *file_name, T *proto) {
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  using google::protobuf::io::CodedInputStream;
  using google::protobuf::io::ZeroCopyOutputStream;
  using google::protobuf::io::CodedOutputStream;
  using google::protobuf::Message;

  int fd = open(file_name, O_RDONLY);
  if (fd == -1) {
    proto = NULL;
    ROS_ERROR("File not found: %s", file_name);
  }

  ZeroCopyInputStream *raw_input = new FileInputStream(fd);
  CodedInputStream *coded_input = new CodedInputStream(raw_input);
  coded_input->SetTotalBytesLimit(kProtoReadBytesLimit, 536870912);

  bool success = proto->ParseFromCodedStream(coded_input);

  delete coded_input;
  delete raw_input;
  close(fd);
  return success;
}
template<class T>
inline bool ReadProtoFromBinaryFile(const std::string &file_name, T *proto) {
  return ReadProtoFromBinaryFile(file_name.c_str(), proto);
}

template<class T>
inline bool ReadYmlFromFile(const char *file_name, T *yml_type);
template<class T>
inline bool ReadYmlFromFile(const std::string &file_name, T *yml_type) {
  return ReadYmlFromFile(file_name.c_str(), yml_type);
}
}

#endif // ROBORTS_COMMON_IO_H