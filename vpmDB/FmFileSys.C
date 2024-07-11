// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmFileSys.H"
#include "FFaLib/FFaOS/FFaFilePath.H"

#include <iostream>
#ifdef FT_HAS_QT
#include <QDateTime>
#include <QFileInfo>
#include <QDir>
#else
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#ifdef FT_HAS_DIRENT
#include <dirent.h>
#endif
#endif
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


std::string FmFileSys::getHomeDir()
{
  char* HOME = getenv("HOME");
  if (HOME) return HOME;

#ifdef FT_HAS_QT
  return QDir::toNativeSeparators(QDir::homePath()).toStdString();
#else
  return "";
#endif
}


unsigned int FmFileSys::getFileSize(const std::string& filename)
{
#ifdef FT_HAS_QT
  QFileInfo aFile(filename.c_str());
  return aFile.size();
#else
  struct stat st;
  if (stat(filename.c_str(),&st))
    return 0;

  return st.st_size;
#endif
}


std::string FmFileSys::fileLastModified(const std::string& filename)
{
#ifdef FT_HAS_QT
  QFileInfo file(filename.c_str());
  return file.lastModified().toString(Qt::TextDate).toStdString();
#else
  struct stat st;
  if (stat(filename.c_str(),&st))
    return "";

  return ctime(&st.st_mtime);
#endif
}


bool FmFileSys::isFile(const std::string& path)
{
  if (path.empty())
    return false;

#ifdef FT_HAS_QT
  QFileInfo info(path.c_str());
  return info.isFile();
#else
  struct stat st;
  if (stat(path.c_str(),&st))
    return false;

  return S_ISREG(st.st_mode);
#endif
}


bool FmFileSys::isDirectory(const std::string& path)
{
  if (path.empty())
    return false;

#ifdef FT_HAS_QT
  QFileInfo info(path.c_str());
  return info.isDir() && info.isReadable();
#else
  struct stat st;
  if (stat(path.c_str(),&st))
    return false;

  return S_ISDIR(st.st_mode);
#endif
}


bool FmFileSys::isReadable(const std::string& filename)
{
  if (filename.empty())
    return false;

#ifdef FT_HAS_QT
  QFileInfo info(filename.c_str());
  return info.isReadable();
#else
  return FmFileSys::isFile(filename);
#endif
}


bool FmFileSys::isWritable(const std::string& filename)
{
  if (filename.empty())
    return false;

#ifdef FT_HAS_QT
  QFileInfo info(filename.c_str());
  return info.isWritable();
#else
  return false;
#endif
}


bool FmFileSys::deleteFile(const std::string& filename)
{
#ifdef FT_HAS_QT
  return QFile::remove(filename.c_str());
#else
  return remove(filename.c_str()) == 0;
#endif
}


bool FmFileSys::renameFile(const std::string& from, const std::string& to)
{
  if (isFile(to))
    FmFileSys::deleteFile(to);

  return rename(from.c_str(),to.c_str()) == 0;
}


bool FmFileSys::copyFile(const std::string& from, const std::string& to)
{
  FILE* inf = fopen(from.c_str(),"rb");
  if (!inf) return false;

  FILE* outf = fopen(to.c_str(),"wb");
  if (!outf)
  {
    fclose(inf);
    return false;
  }

  size_t r;
  char buf[16384];
  while ((r = fread(buf,1,sizeof(buf),inf)) > 0 && fwrite(buf,1,r,outf) == r);

  bool ierr = (ferror(inf) == 0)  & (fclose(inf) == 0);
  bool oerr = (ferror(outf) == 0) & (fclose(outf) == 0);
  if (!oerr) deleteFile(to); // delete incomplete output files

  return ierr && oerr;
}


bool FmFileSys::copyFile(const std::string& filename,
                         const std::string& from, const std::string& to)
{
  return FmFileSys::copyFile(FFaFilePath::appendFileNameToPath(from,filename),
                             FFaFilePath::appendFileNameToPath(to,filename));
}


bool FmFileSys::verifyDirectory(const std::string& dirName, bool create)
{
  if (dirName.empty())
    return true; // an empty dirName is interpreted as current working directory

#ifdef FT_HAS_QT
  QDir checkDir(dirName.c_str());
  if (checkDir.exists())
    return true;
  else if (!create)
    return false;

  // Check if it is a file
  QFileInfo fi(dirName.c_str());
  if (fi.exists())
    return false;

  // Try to create
  QDir newDir;
  return newDir.mkdir(dirName.c_str());
#else
  if (FmFileSys::isDirectory(dirName))
    return true;
  else if (!create)
    return false;
  else
    return mkdir(dirName.c_str(),0777) == 0;
#endif
}


/*!
  \brief Static helper for extracting file names from a directory.
  \param files List of found file (or directory) names
  \param[in] path Full pathname of the directory to search for files in
  \param[in] ext File extension(s) to search for.
             If NULL, search for directories.
  \param[in] filter File name filter. If NULL, no filtering.
  \param[in] fullPath If \e true, return full path names,
             otherwise relative to \a path
*/

static bool get_files (std::vector<std::string>& files,
                       const char* path, const char* ext,
                       const char* filter, bool fullPath = true)
{
  size_t existingFiles = files.size();
#ifdef FT_HAS_QT
  std::string namefilter("*");
  if (ext && filter)
    namefilter = filter + std::string(".") + ext;
  else if (filter)
    namefilter = filter;
  else if (ext)
  {
    if (strstr(ext,"*"))
      namefilter = ext;
    else
      namefilter = std::string("*.") + ext;
  }
  QDir rdbDirFiles(path, namefilter.c_str(),
                   QDir::Name | QDir::IgnoreCase,
                   ext ? QDir::Files : QDir::Dirs | QDir::NoDotAndDotDot);
  if (!rdbDirFiles.exists()) return false;

  files.reserve(existingFiles + rdbDirFiles.count());
  for (unsigned int i = 0; i < rdbDirFiles.count(); i++)
    files.push_back(rdbDirFiles[i].toStdString());
#elif defined(FT_HAS_DIRENT)
  // Non-Qt version, using dirent
  DIR* dir = opendir(path);
  if (!dir)
  {
    perror(path);
    return false;
  }

  // No name filtering implemented yet (only extensions)
  if (filter)
    std::cout <<"  ** FmFileSys::getDirs(): Ignoring filter \""<< filter
              <<"\" (not implemented)."<< std::endl;

  struct dirent* ent;
  while ((ent = readdir(dir)))
    if (ent->d_name[0] != '.')
    {
      if (ent->d_type == DT_DIR && !ext)
        // Found a directory
        files.push_back(ent->d_name);
      else if (ent->d_type != DT_DIR && ext)
      {
        // Found a file, check its extension
        std::string fext = FFaFilePath::getExtension(ent->d_name);
        if (std::string(ext).find(fext) != std::string::npos)
          files.push_back(ent->d_name);
      }
    }

  closedir(dir);
#endif

  if (fullPath)
    for (size_t i = existingFiles; i < files.size(); i++)
      FFaFilePath::makeItAbsolute(files[i], path);

  return files.size() > existingFiles;
}


bool FmFileSys::getDirs(std::vector<std::string>& foundDirs,
                        const std::string& searchPath,
                        const char* filter, bool fullPath)
{
  foundDirs.clear();
  return get_files(foundDirs,searchPath.c_str(),NULL,filter,fullPath);
}


bool FmFileSys::getFiles(std::vector<std::string>& foundFiles,
                         const std::string& searchPath,
                         const char* filter, bool fullPath)
{
  if (filter)
  {
    foundFiles.clear();
    return get_files(foundFiles,searchPath.c_str(),filter,NULL,fullPath);
  }

  // Do a recursive search in subdirectories when no file filter
  bool gotFiles = false;
  std::vector<std::string> foundDirs;
  if (get_files(foundDirs,searchPath.c_str(),NULL,NULL,true))
    for (const std::string& subDir : foundDirs)
      gotFiles |= FmFileSys::getFiles(foundFiles,subDir,NULL,fullPath);

  return get_files(foundFiles,searchPath.c_str(),"*",NULL,fullPath) || gotFiles;
}


int FmFileSys::getNextDirIncrement(const std::string& dirName,
                                   const std::string& baseDirName)
{
  std::vector<std::string> dirs;
  std::string namefilter = baseDirName + "*";
  if (!get_files(dirs,dirName.c_str(),NULL,namefilter.c_str()))
    return 1;

  int retvar = 1;
  for (const std::string& dir : dirs)
  {
    size_t us = dir.rfind("_");
    if (us+1 < dir.size() && dir.substr(0,us) == baseDirName)
    {
      int ver = atoi(dir.substr(us+1).c_str());
      if (ver+1 > retvar) retvar = ver+1;
    }
  }

  return retvar;
}


int FmFileSys::getNextIncrement(const std::string& dirName,
                                const char* extension, int startIncr,
                                const char* filter)
{
  std::vector<std::string> files;
  if (!get_files(files,dirName.c_str(),extension,filter))
    return startIncr;

  int retvar = startIncr;
  for (const std::string& fileName : files)
  {
    std::string basename = FFaFilePath::getBaseName(fileName);
    size_t us = basename.rfind("_");
    if (us+1 < basename.size())
    {
      int ver = atoi(basename.substr(us+1).c_str());
      if (ver+1 > retvar) retvar = ver+1;
    }
  }

  return retvar;
}


int FmFileSys::getNextIncrement(const std::vector<std::string>& dirNames,
                                const char* extension)
{
  int retvar = 1;
  for (const std::string& dir : dirNames)
    retvar = FmFileSys::getNextIncrement(dir,extension,retvar);
  return retvar;
}


int FmFileSys::removeDir(const std::string& dirName, bool removeFiles)
{
  std::vector<std::string> files;
#ifdef FT_HAS_QT
  QDir dir(dirName.c_str(), "*", QDir::Name | QDir::IgnoreCase,
           QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);
  if (!dir.exists()) return -1;
  files.reserve(dir.count());
  for (unsigned int i = 0; i < dir.count(); i++)
    files.push_back(dir[i].toStdString());
#elif defined(FT_HAS_DIRENT)
  DIR* dir = opendir(dirName.c_str());
  if (!dir) return -1;
  struct dirent* ent;
  while ((ent = readdir(dir)))
    if (ent->d_name[0] != '.')
      files.push_back(ent->d_name);
  closedir(dir);
#endif

  int ret, ndel = 0;
  for (std::string& fName : files)
  {
    if (FmFileSys::isDirectory(FFaFilePath::makeItAbsolute(fName,dirName)))
      ret = FmFileSys::removeDir(fName,removeFiles);
    else if (removeFiles)
    {
      ret = FmFileSys::deleteFile(fName) ? 1 : -1;
      if (ret < 0)
        std::cerr <<" *** Could not delete file "<< fName << std::endl;
    }
    else
      ret = -1;
    if (ret < 0 && ndel >= 0)
      ndel = ret;
    else if (ret < 0 || ndel >= 0)
      ndel += ret;
  }
  if (ndel < 0) return ndel;

#if FT_HAS_QT > 4
  ret = dir.rmdir(dirName.c_str()) ? ndel : -1;
#elif defined(FT_HAS_QT)
  ret = dir.rmdir(".") ? ndel : -1;
#else
  ret = rmdir(dirName.c_str());
  if (ret == 0) ret = ndel;
#endif
  if (ret < 0)
    std::cerr <<" *** Could not delete directory "<< dirName << std::endl;
  return ret;
}
