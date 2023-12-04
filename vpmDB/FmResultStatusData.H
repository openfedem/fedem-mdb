// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_RESULT_STATUS_DATA_H
#define FM_RESULT_STATUS_DATA_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>


/*!
  A class to store a directory structure based on a <taskname>_####
  naming convension. Contains a recursive hierarcy of itself.
*/

class FmResultStatusData
{
public:
  FmResultStatusData(const std::string& name = "noname")
    : myTaskName(name), myTaskVer(1u) {}
  FmResultStatusData(const FmResultStatusData& ref) { this->copy(&ref); }
  ~FmResultStatusData();

  FmResultStatusData& operator=(const FmResultStatusData& ref);
  bool operator==(const FmResultStatusData& ref) const;

  void clear();
  void copy(const FmResultStatusData* obj);

  bool addFile(const std::string& filename);
  void addFiles(const std::vector<std::string>& files);

  bool removeFile(const std::string& filename);
  void removeFiles(const std::vector<std::string>& files);
  void removeFiles(const std::set<std::string>& files);

  bool ignoreFile(const std::string& filename);
  void ignoreFiles(const std::vector<std::string>& files);

  //! \brief Fills this RSD with data from the specified \a rdbDir on disk.
  //! \return Total number of files found.
  size_t syncFromRDB(const std::string& rdbDir,
                     const std::string& taskName, int taskVer,
                     std::set<std::string>* obsoleteFiles = NULL);

  const std::set<std::string>& getFileSet() const { return myFiles; }

  FmResultStatusData* addSubTask(const std::string& name);
  FmResultStatusData* getSubTask(const std::string& name, int ver = 0) const;

  bool isSubTask(const std::string& name) const;

  bool setPath(const std::string& path) { return this->newPath(path,myPath.size()); }
  void setTaskName(const std::string& taskName) { myTaskName = taskName; }
  bool setTaskVer(int taskVer);
  void incrementTaskVer() { this->clear(); myTaskVer++; }

  std::string getCurrentTaskDirName(bool fullPath = false,
                                    bool checkTask = false) const;
  const std::string& getPath() const { return myPath; }
  const std::string& getTaskName() const { return myTaskName; }
  bool isNamed() const { return !myTaskName.empty() && myTaskName != "noname"; }
  int getTaskVer() const { return myTaskVer; }

  bool isEmpty(bool resultFilesOnly = false) const;

  void write(std::ostream& os) const;
  void read(std::istream& is);

  bool hasFileNames(const std::string& filter = "",
                    bool recursive = true) const;
  bool getAllFileNames(std::set<std::string>& fileNames,
                       const std::string& filter = "",
                       bool withPath = true, bool recursive = true) const;
  std::string getFileNames(const std::string& filter,
                           bool withPath = true, bool recur = true) const;
  bool getFrsFiles(std::set<std::string>& frsFiles,
                   const std::string& rdbResultGroup = "",
                   bool silence = false) const;

  void getAllDirNames(std::set<std::string>& dirNames) const;

  static int getTaskVer(const std::string& baseName, size_t* pos = NULL);
  static bool splitRDBName(const std::string& baseName,
                           std::string& taskName, int& taskVer);

private:
  // Recursive private methods for populating an RSD instance
  bool newPath(const std::string& prefix, size_t lenP);
  void processTokens(const std::vector<std::string>& tokens);
  size_t syncDisk(const std::string& rdbDir,
                  const std::string& taskName, int taskVer,
                  const std::string& nameFilter,
                  std::set<std::string>* obsoleteFiles);

private:
  typedef std::map<std::string,FmResultStatusData*> FmTaskMap;
  typedef FmTaskMap::const_iterator                 FmTaskIter;

  std::string           myPath;
  std::string           myTaskName;
  unsigned short int    myTaskVer;
  std::set<std::string> myFiles;
  FmTaskMap             mySubTasks;

  friend std::ostream& operator<<(std::ostream& os, const FmResultStatusData& field);
  friend std::istream& operator>>(std::istream& is, FmResultStatusData& field);
};

#endif
