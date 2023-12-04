// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmFileSys.H"
#include <iostream>


int main(int argc, const char** argv)
{
  if (argc < 2) return 1;

  std::vector<std::string> files;
  FmFileSys::getFiles(files,argv[1],NULL,true);
  std::cout <<"FmFileSys::getFiles(): "<< argv[1];
  for (const std::string& fn : files) std::cout <<"\n\t"<< fn;
  bool delFile = argc > 2 ? argv[2][0] == 'y' : true;
  std::cout <<"\nFmFileSys::removeDir(): "<< argv[1]
            <<" "<< std::boolalpha << delFile << std::endl;
  int ndel = FmFileSys::removeDir(argv[1],delFile);
  if (ndel >= 0)
    std::cout <<"Success, "<< ndel <<" files deleted."<< std::endl;
  else
    std::cout <<"Failure to delete "<< -ndel <<" files."<< std::endl;

  return ndel < 0 ? ndel : 0;
}
