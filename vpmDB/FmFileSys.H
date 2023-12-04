// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmFileSys.H
  \brief Global functions for manipulation of files on disk.
*/

#ifndef FM_FILE_SYS_H
#define FM_FILE_SYS_H

#include <string>
#include <vector>


namespace FmFileSys //! File system utilities
{
  unsigned int getFileSize(const std::string& filename);

  std::string fileLastModified(const std::string& filename);

  std::string getHomeDir();

  bool isFile(const std::string& path);
  bool isDirectory(const std::string& path);

  bool isReadable(const std::string& filename);
  bool isWritable(const std::string& filename);

  bool deleteFile(const std::string& filename);
  bool renameFile(const std::string& from, const std::string& to);
  bool copyFile(const std::string& from, const std::string& to);
  bool copyFile(const std::string& filename,
                const std::string& from, const std::string& to);

  //! \brief Check that the directory \a dirName exists - if not create it.
  bool verifyDirectory(const std::string& dirName, bool create = true);

  //! \brief Recursively remove the directory \a dirName.
  int removeDir(const std::string& dirName, bool removeFiles = true);

  //! \brief File name checks - get unique number on a directory.
  int getNextDirIncrement(const std::string& dirName,
                          const std::string& baseDirName);
  //! \brief File name checks - get unique number on a file.
  int getNextIncrement(const std::string& dirName, const char* extension,
                       int startInc = 1, const char* filter = NULL);
  //! \brief File name checks - get unique number on files.
  int getNextIncrement(const std::vector<std::string>& dirNames,
                       const char* extension);

  //! \brief Find all subdirectories in a named directory.
  bool getDirs(std::vector<std::string>& foundDirs,
               const std::string& searchPath,
               const char* filter = NULL, bool fullPath = false);

  //! \brief Find all files in a named directory.
  bool getFiles(std::vector<std::string>& foundFiles,
                const std::string& searchPath,
                const char* filter = NULL, bool fullPath = false);
}

#endif
