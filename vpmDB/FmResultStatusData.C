// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <cstdlib>
#include <cctype>

#include "FFaLib/FFaString/FFaTokenizer.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#include "vpmDB/FmResultStatusData.H"
#include "vpmDB/FmFileSys.H"


FmResultStatusData::~FmResultStatusData()
{
  for (FmTaskMap::value_type& task : mySubTasks)
    delete task.second;
}


FmResultStatusData& FmResultStatusData::operator=(const FmResultStatusData& ref)
{
  if (this != &ref)
    this->copy(&ref);

  return *this;
}


bool FmResultStatusData::operator==(const FmResultStatusData& ref) const
{
  if (this == &ref) return true;

  // Note: We don't compare myPath, since that is only internal information
  // which is generated automatically and not stored on the model file
  if (this->myTaskName != ref.myTaskName) return false;
  if (this->myTaskVer  != ref.myTaskVer)  return false;
  if (this->isEmpty()  && ref.isEmpty())  return true;

  if (this->myFiles.size() != ref.myFiles.size()) return false;
  if (this->mySubTasks.size() != ref.mySubTasks.size()) return false;

  std::set<std::string>::const_iterator i, j = ref.myFiles.begin();
  for (i = this->myFiles.begin(); i != this->myFiles.end(); ++i, ++j)
    if (*i != *j) return false;

  FmTaskIter is, js = ref.mySubTasks.begin();
  for (is = this->mySubTasks.begin(); is != this->mySubTasks.end(); ++is, ++js)
  {
    if (!(is->first  == js->first))  return false;
    if (!(is->second == js->second)) return false;
  }

  return true;
}


/*!
  Adds the file named \a fileName to the RSD data structure by chopping off the
  first directory name in the path, adding it as sub-task, and then adding the
  rest of the \a fileName to that sub-task in a recursive manner.
*/

bool FmResultStatusData::addFile(const std::string& fileName)
{
  std::string taskDir = this->getCurrentTaskDirName(false,true);
  if (taskDir.empty()) return false; // Logic error: adding file to unnamed task

  // Need to erase the first part of the string, if it contains taskDir
  size_t p = fileName.find(taskDir);
  std::string subName = fileName.substr(p == std::string::npos ? 0 : p+1+taskDir.size());

  // Add the file to current task if no path-separators left
  size_t splitPos = subName.find(FFaFilePath::getPathSeparator());
  if (splitPos == std::string::npos)
    return myFiles.insert(subName).second;

  // The subName still contains path-separators, add it to a sub-task
  std::string taskName; int taskVer;
  splitRDBName(subName.substr(0,splitPos),taskName,taskVer);
  FmResultStatusData* subRSD = this->addSubTask(taskName);
  return subRSD ? subRSD->addFile(subName.substr(splitPos+1)) : false;
}


void FmResultStatusData::addFiles(const std::vector<std::string>& fileNames)
{
  for (const std::string& fileName : fileNames) this->addFile(fileName);
}


/*!
  Removes the file with full path \a fileName from the task hierarchy.
*/

bool FmResultStatusData::removeFile(const std::string& fileName)
{
  std::string taskDir = this->getCurrentTaskDirName();

  // Need to erase the first part of the string, if it contains taskDir
  size_t p = taskDir.empty() ? std::string::npos : fileName.find(taskDir);
  std::string subName = fileName.substr(p == std::string::npos ? 0 : p+1+taskDir.size());

  // Try to remove the file from current task if no path-separators left
  size_t splitPos = subName.find(FFaFilePath::getPathSeparator());
  if (splitPos == std::string::npos)
    return myFiles.erase(subName) > 0;

  // The subName still contains path-separators, remove it from a sub-task
  std::string taskName; int taskVer;
  splitRDBName(subName.substr(0,splitPos),taskName,taskVer);
  FmResultStatusData* subTask = this->getSubTask(taskName);
  if (subTask)
    return subTask->removeFile(subName.substr(splitPos+1));
  else
    return false;
}


void FmResultStatusData::removeFiles(const std::vector<std::string>& files)
{
  if (this->isEmpty()) return;

  for (const std::string& fileName : files)
    if (!this->removeFile(fileName))
      ListUI <<"  -> Could not remove "<< fileName <<" from RSD.\n";
}


void FmResultStatusData::removeFiles(const std::set<std::string>& files)
{
  if (this->isEmpty()) return;

  for (const std::string& fileName : files)
    if (!this->removeFile(fileName))
      ListUI <<"  -> Could not remove "<< fileName <<" from RSD.\n";
}


/*!
  Ignores the file with full path \a fileName by incrementing the task version
  of the sub RSD that normally should contain that file.
*/

bool FmResultStatusData::ignoreFile(const std::string& fileName)
{
  std::string taskDir = this->getCurrentTaskDirName();

  // Need to erase the first part of the string, if it contains taskDir
  size_t p = taskDir.empty() ? std::string::npos : fileName.find(taskDir);
  std::string subName = fileName.substr(p == std::string::npos ? 0 : p+1+taskDir.size());

  // Check if there are more path separators in subName
  size_t splitPos = subName.find(FFaFilePath::getPathSeparator());
  if (splitPos == std::string::npos)
  {
    // We are in the correct RSD containing this file
    // Clear its content and increment the task version
    std::set<std::string> oldFiles;
    this->getAllFileNames(oldFiles);
    this->incrementTaskVer();
    if (!oldFiles.empty())
    {
      ListUI <<"  -> Ignoring existing files in "<< taskDir <<"\n";
      for (const std::string& file : oldFiles) ListUI <<"\t"<< file <<"\n";
    }
    return true;
  }

  // The subName still contains path-separators, check for a matching sub-task
  std::string taskName; int taskVer = 0;
  splitRDBName(subName.substr(0,splitPos),taskName,taskVer);
  FmResultStatusData* subTask = this->getSubTask(taskName,taskVer);
  if (subTask) return subTask->ignoreFile(subName.substr(splitPos+1));

  // The subName did not match an existing sub-task,
  // check if another sub-task with a lower task version exist
  subTask = this->getSubTask(taskName);
  if (!subTask) return false;
  if (subTask->getTaskVer() > taskVer) return false;

  // A sub-task with a lower task version exists,
  // clear its content and increment the task version
  std::set<std::string> oldFiles;
  subTask->getAllFileNames(oldFiles);
  taskDir = subTask->getCurrentTaskDirName();
  subTask->clear();
  subTask->setTaskVer(taskVer+1);
  if (!oldFiles.empty())
  {
    ListUI <<"  -> Ignoring existing files in "<< taskDir <<"\n";
    for (const std::string& file : oldFiles) ListUI <<"\t"<< file <<"\n";
  }
  return true;
}


void FmResultStatusData::ignoreFiles(const std::vector<std::string>& fileNames)
{
  for (const std::string& fileName : fileNames) this->ignoreFile(fileName);
}


bool FmResultStatusData::isSubTask(const std::string& name) const
{
  return mySubTasks.empty() ? false : mySubTasks.find(name) != mySubTasks.end();
}


bool FmResultStatusData::isEmpty(bool resultFilesOnly) const
{
  if (resultFilesOnly)
    return !this->hasFileNames("res") && !this->hasFileNames("frs");
  else if (!myFiles.empty())
    return false;

  // Check for file in subtasks also
  for (const FmTaskMap::value_type& task : mySubTasks)
    if (!task.second->isEmpty(resultFilesOnly))
      return false;

  return true; // this task and all its subtasks were empty
}


FmResultStatusData* FmResultStatusData::addSubTask(const std::string& subTaskName)
{
  FmResultStatusData* info = new FmResultStatusData(subTaskName);
  if (!info->setPath(this->getCurrentTaskDirName(true,true)))
  {
    // Bugfix #511: Don't create sub-tasks of unnamed tasks.
    delete info;
    return NULL;
  }

  std::pair<FmTaskMap::iterator,bool> p;
  p = mySubTasks.insert(std::make_pair(subTaskName,info));
  if (!p.second) delete info;

  return p.first->second;
}


FmResultStatusData* FmResultStatusData::getSubTask(const std::string& name,
                                                   int ver) const
{
  if (name.empty() || ver > 9999)
    return NULL; // out of range

  FmTaskIter it = mySubTasks.find(name);
  if (it == mySubTasks.end()) return NULL;

  return ver < 1 || ver == it->second->getTaskVer() ? it->second : NULL;
}


int FmResultStatusData::getTaskVer(const std::string& baseName, size_t* pos)
{
  // The task version number is after the last '_' character
  size_t last_pos = baseName.find_last_of('_');
  if (last_pos >= baseName.size()-1)
    return -1; // no '_' in the string

  for (size_t j = last_pos+1; j < baseName.size(); j++)
    if (!isdigit(baseName[j]))
      return -j; // no number

  if (pos) *pos = last_pos;
  return atoi(baseName.substr(last_pos+1).c_str());
}


bool FmResultStatusData::setTaskVer(int taskVer)
{
  if (taskVer < 0 || taskVer > 9999)
  {
    ListUI <<"  -> Task version "<< taskVer
           <<" is out of valid range [0,9999], resetting to 1\n";
    myTaskVer = 1u;
    return false;
  }

  myTaskVer = taskVer;
  return true;
}


bool FmResultStatusData::splitRDBName(const std::string& baseName,
                                      std::string& taskName, int& taskVer)
{
  size_t last_pos = baseName.size();
  if ((taskVer = getTaskVer(baseName,&last_pos)) < 0)
    return false;

  taskName.assign(baseName,0,last_pos);

  return true;
}


/*!
  Replaces the first \a lenP characters of the current path of this task
  and all its sub tasks by \a prefix.
*/

bool FmResultStatusData::newPath(const std::string& prefix, size_t lenP)
{
  if (prefix.empty())
    return false;
  else if (lenP >= myPath.size())
  {
    if (prefix == myPath) return true;
    myPath = prefix;
  }
  else if (lenP == 0)
    myPath.insert(0,prefix+FFaFilePath::getPathSeparator());
  else
    myPath.replace(0,lenP,prefix);

  for (FmTaskMap::value_type& task : mySubTasks)
    if (!task.second->newPath(prefix,lenP))
      return false;

  return true;
}


/*!
  Returns the string "taskName_####" - identifier for the current directory.
  The full path of the directory may optionally be prepended.
*/

std::string FmResultStatusData::getCurrentTaskDirName(bool fullPath, bool checkTask) const
{
  if (myTaskName.empty() || (checkTask && myTaskName == "noname"))
  {
    // Bugfix #511: Return a blank name for yet unnamed tasks.
#ifdef FM_DEBUG
    // Print message (to console only) in case it is due to some logic error.
    std::cerr <<"  ** FmResultStatusData::getCurrentTaskDirName: Unnamed task "
              << myTaskVer <<"\n     path = "<< myPath << std::endl;
#endif
    return "";
  }

  char num[8];
  snprintf(num,8,"_%04hu",myTaskVer);
  if (fullPath)
    return FFaFilePath::appendFileNameToPath(myPath, myTaskName + num);
  else
    return myTaskName + num;
}


/*!
  Checks if the RSD has any file names.
*/

bool FmResultStatusData::hasFileNames(const std::string& filter, bool recursive) const
{
  for (const std::string& file : myFiles)
    if (filter.empty() || FFaFilePath::isExtension(file,filter))
      return true;

  if (recursive)
    for (const FmTaskMap::value_type& task : mySubTasks)
      if (task.second->hasFileNames(filter))
        return true;

  return false;
}


/*!
  Returns a sorted set of all file names in the RSD.
*/

bool FmResultStatusData::getAllFileNames(std::set<std::string>& fileNames,
                                         const std::string& filter,
                                         bool withPath, bool recursive) const
{
  std::string path = withPath ? this->getCurrentTaskDirName(true) : "";

  for (const std::string& file : myFiles)
    if (filter.empty() || FFaFilePath::isExtension(file,filter))
      fileNames.insert(withPath ? FFaFilePath::appendFileNameToPath(path,file) : file);

  if (recursive)
    for (const FmTaskMap::value_type& task : mySubTasks)
      task.second->getAllFileNames(fileNames,filter,withPath);

  return !fileNames.empty();
}


/*!
  Returns all file names in the RSD as a ";"-separated list
*/

std::string FmResultStatusData::getFileNames(const std::string& filter,
                                             bool withPath, bool recur) const
{
  std::set<std::string> fileNames;
  this->getAllFileNames(fileNames,filter,withPath,recur);

  std::string fileList;
  for (const std::string& fileName : fileNames)
    if (fileList.empty())
      fileList.assign(fileName);
    else
      fileList.append(";"+fileName);

  return fileList;
}


bool FmResultStatusData::getFrsFiles(std::set<std::string>& frsFiles,
                                     const std::string& rdbResultGroup,
                                     bool silence) const
{
  if (this->isEmpty(true))
  {
    if (!silence)
      ListUI <<"===> Empty RSD, no result groups present.\n";
    return false;
  }

  // Check if we have a sub task with the given name
  if (rdbResultGroup.empty())
    return this->getAllFileNames(frsFiles,"frs");
  else if (!this->isSubTask(rdbResultGroup))
  {
#ifdef FM_DEBUG
    if (!silence)
      ListUI <<"===> No result group named " + rdbResultGroup + " present.\n";
#endif
    return false;
  }

  // Get result file names in the sub task
  FmResultStatusData* subRSD = this->getSubTask(rdbResultGroup);
  return subRSD ? subRSD->getAllFileNames(frsFiles,"frs") : false;
}


void FmResultStatusData::getAllDirNames(std::set<std::string>& dirNames) const
{
  dirNames.insert(this->getCurrentTaskDirName(true));

  for (const FmTaskMap::value_type& task : mySubTasks)
    task.second->getAllDirNames(dirNames);
}


/*!
  Clears the contents of the RSD.
  Preserves the path, taskName and version though.
*/

void FmResultStatusData::clear()
{
  myFiles.clear();

  for (FmTaskMap::value_type& task : mySubTasks)
  {
    task.second->clear();
    delete task.second;
  }

  mySubTasks.clear();
}


/*!
  Copies information from \a obj to this RSD object.
  All original data in this object are lost.
*/

void FmResultStatusData::copy(const FmResultStatusData* obj)
{
  this->clear();

  myPath = obj->myPath;
  myTaskName = obj->myTaskName;
  myTaskVer = obj->myTaskVer;
  myFiles = obj->myFiles;

  for (const FmTaskMap::value_type& task : obj->mySubTasks)
    mySubTasks[task.first] = new FmResultStatusData(*task.second);
}


size_t FmResultStatusData::syncFromRDB(const std::string& rdbDir,
                                       const std::string& taskName, int taskVer,
                                       std::set<std::string>* obsoleteFiles)
{
  static std::string myFilter;
  if (myFilter.empty())
  {
    // Set up list of extensions for all file types we will be looking for
    const char* extensions[] = {
      "fao", "fco", "fop",
      "fmm", "ftl", "fsi",
      "fmx", "fsm", "frs", "res",
      "asc", "dac",
      "fpp", "fef",
      "ipt", "wnd", "elm",
      NULL
    };

    // Compose the name filter
    for (const char** q = extensions; *q; ++q)
      myFilter += std::string(" *.") + std::string(*q);
  }

  // Invoke the recursive method filtering with the interesting file extensions
  return this->syncDisk(rdbDir,taskName,taskVer,myFilter,obsoleteFiles);
}


size_t FmResultStatusData::syncDisk(const std::string& rdbDir,
                                    const std::string& taskName, int taskVer,
                                    const std::string& nameFilter,
                                    std::set<std::string>* obsoleteFiles)
{
#if FM_DEBUG > 5
  std::cout <<"\nFmResultStatusData::syncFromRDB()\n\t"
            << rdbDir <<"\n\t["<< taskName <<"] "<< taskVer
            <<"\n"<< nameFilter << std::endl;
#endif

  this->clear();
  this->setTaskName(taskName);
  this->setTaskVer(taskVer);
  if (rdbDir.empty())
    return 0;

  // Find files on disk
  std::vector<std::string> rdbDirFiles;
  if (FmFileSys::getFiles(rdbDirFiles,rdbDir,nameFilter.c_str()))
    for (const std::string& file : rdbDirFiles)
    {
      this->addFile(file);
#if FM_DEBUG > 5
      std::cout <<"\t"<< file << std::endl;
#endif
    }

  // Check the sub-directories, if any
  size_t nFiles = rdbDirFiles.size();
  std::vector<std::string> rdbDirDirs;
  if (!FmFileSys::getDirs(rdbDirDirs,rdbDir))
    return nFiles;

  for (std::string& dir : rdbDirDirs)
  {
    // Create a new (or find existing) RSD for the sub-directory
    std::string stName; int stVer = -1;
    if (!splitRDBName(dir,stName,stVer)) continue;
    FmResultStatusData* subRSD = this->addSubTask(stName);
    if (!subRSD) continue;

    // Check if the new RSD is empty, or has a lower task id
    FFaFilePath::makeItAbsolute(dir,rdbDir);
    if (subRSD->isEmpty())
      nFiles += subRSD->syncDisk(dir,stName,stVer,nameFilter,obsoleteFiles);
    else if (subRSD->getTaskVer() < stVer)
    {
      // The task version of this subRSD is less than we have found on disk.
      // This means that we should remove all current files in subRSD and insert
      // the correct task version and the new files found on disk instead.
      if (obsoleteFiles) subRSD->getAllFileNames(*obsoleteFiles);
      nFiles += subRSD->syncDisk(dir,stName,stVer,nameFilter,obsoleteFiles);
    }
  }

#if FM_DEBUG > 5
  if (obsoleteFiles && !obsoleteFiles->empty())
  {
    std::cout <<"\nobsoleteFiles:";
    for (const std::string& file : *obsoleteFiles)
      std::cout <<"\n"<< file;
    std::cout << std::endl;
  }
#endif
  return nFiles;
}


void FmResultStatusData::processTokens(const std::vector<std::string>& tokens)
{
  if (tokens.size() < 2)
  {
    ListUI <<"  -> Syntax error in result status data - check model file.\n";
    return;
  }

  // First two are RSD info
  this->setTaskName(tokens.front());
  this->setTaskVer(atoi(tokens[1].c_str()));

  if (tokens.size() == 2)
    return; // Empty RSD, stop processing

  std::string taskDir = this->getCurrentTaskDirName(true,true);
  if (taskDir.empty()) return; // Logic error: adding file to unnamed task

  for (size_t i = 2; i < tokens.size(); i++)
    if (tokens[i][0] == '<')
    {
      // The first char is a '<', create a new RSD entry and put it in place
      FmResultStatusData* newRSD = new FmResultStatusData();
      newRSD->setPath(taskDir);
      newRSD->processTokens(FFaTokenizer(tokens[i],'<','>',','));
      mySubTasks[newRSD->getTaskName()] = newRSD;
    }
    else
      myFiles.insert(tokens[i]);
}


void FmResultStatusData::write(std::ostream& os) const
{
  static int indent = 0;
  for (int i = 0; i < indent; i++)
    os <<"    ";

  os <<"<\"" << myTaskName <<"\","<< myTaskVer;
  for (const std::string& file : myFiles)
    os <<",\""<< file <<"\"";

  indent++;
  for (const FmTaskMap::value_type& task : mySubTasks)
  {
    os <<",";
    if (indent != 0) os << std::endl;
    task.second->write(os);
  }
  indent--;

  os <<">";
}


void FmResultStatusData::read(std::istream& is)
{
  char c;
  while (is.get(c) && isspace(c)); // read to non-whitespace

  if (c == '<')
    this->processTokens(FFaTokenizer(is,'<','>',','));
}


std::ostream& operator<<(std::ostream& os, const FmResultStatusData& rsd)
{
  rsd.write(os);
  return os;
}


std::istream& operator>>(std::istream& is, FmResultStatusData& rsd)
{
  rsd.read(is);
  return is;
}
