// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmThreshold.H"


bool FmThreshold::operator==(const FmThreshold& a) const
{
  if (this == &a) return true;

  return (active == a.active &&
          threshold == a.threshold &&
          min == a.min &&
          skip == a.skip &&
          severity == a.severity);
  // The description we don't care about
}


std::ostream& operator<<(std::ostream& os, const FmThreshold& fld)
{
  os << fld.threshold <<" "<< fld.severity <<" "
     << fld.min <<" "<< fld.skip <<" "<< fld.active;
  if (!fld.description.empty())
    os <<"\n\""<< fld.description <<"\"";
  return os;
}


std::istream& operator>>(std::istream& is, FmThreshold& fld)
{
  is >> fld.threshold >> fld.severity >> fld.min >> fld.skip >> fld.active;
  fld.description.clear();
  if (!is) return is;

  char c, tmp[BUFSIZ];
  while (is.get(c) && c != '\"');
  do
  {
    if (c != '\"') is.putback(c);
    is.get(tmp,BUFSIZ-1,'\"');
    fld.description += tmp;
  }
  while (is.get(c) && c != '\"');

  return is;
}


void FmThreshold::writeAppJson(std::ostream& os,
                               const std::string& tag, const std::string src,
                               size_t indent) const
{
  std::string newline = "\n" + std::string(indent,' ');
  os <<"{"
     << newline <<"  \"code\": {"
     << newline <<"    \"code\": \"" << severity <<"\""
     << newline <<"  },";
  char csev = 'A' + (severity - LOW);
  os << newline <<"  \"severity\": {"
     << newline <<"    \"code\": \""<< csev <<"_"<< severity <<"\""
     << newline <<"  },"
     << newline <<"  \"type\": \"com.sap.newton.StructuralLoadEvent\",";
  if (!description.empty())
    os << newline <<"  \"description\": \""<< description <<"\",";
  os << newline <<"  \"condition\": \"" << tag <<" < "<< threshold <<"\","
     << newline <<"  \"context\": {"
     << newline <<"    \"minInterval\": "<< min <<","
     << newline <<"    \"skipInterval\": "<< skip <<","
     << newline <<"    \"threshold\": "<< threshold
     << newline <<"  },"
     << newline <<"  \"property\": \""<< tag <<"\","
     << newline <<"  \"source\": \""<< src <<"\""
     << newline <<"}";
}
