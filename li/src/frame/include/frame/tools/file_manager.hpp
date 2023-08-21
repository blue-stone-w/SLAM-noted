/*
Description: 读写文件管理
Author     : Wang Junpeng
date       : 
*/

#ifndef FRAME_TOOLS_FILE_MANAGER_HPP_
#define FRAME_TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace frame {
  class FileManager {
    public:
      static bool CreateFile(std::ofstream& ofs, std::string file_path);
      static bool InitDirectory(std::string directory_path, std::string use_for);
      static bool CreateDirectory(std::string directory_path, std::string use_for);
      static bool CreateDirectory(std::string directory_path);
  };
}

#endif