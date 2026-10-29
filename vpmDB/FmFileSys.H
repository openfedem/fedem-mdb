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
  //! \brief Returns the number of bytes in the file \a filename.
  unsigned int getFileSize(const std::string& filename);

  //! \brief Returns the date and time of last change of the file \a filename.
  std::string fileLastModified(const std::string& filename);

  //! \brief Returns the home directory of the running process.
  std::string getHomeDir();

  //! \brief Returns \e true if \a path is an existing file.
  bool isFile(const std::string& path);
  //! \brief Returns \e true if \a path is an existing directory.
  bool isDirectory(const std::string& path);

  //! \brief Returns \e true if \a filename can be opened for reading.
  bool isReadable(const std::string& filename);
  //! \brief Returns \e true if \a filename can be opened for writing.
  bool isWritable(const std::string& filename);

  //! \brief Deletes the file \a filename from disk.
  bool deleteFile(const std::string& filename);
  //! \brief Rename the file named \a from to \a to.
  bool renameFile(const std::string& from, const std::string& to);
  //! \brief Copies the file named \a from to \a to.
  bool copyFile(const std::string& from, const std::string& to);
  //! \brief Copies the file \a filename in directory \a from to \a to.
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
