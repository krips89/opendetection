//
// Created by sarkar on 09.06.15.
//

#ifndef OPENDETECTION_UTILS_H
#define OPENDETECTION_UTILS_H

template<typename T>
string toString(T Number)
{
  ostringstream ss;
  ss << Number;
  return ss.str();
}

static string getTexfileinObj(string objfilename)
{

  boost::filesystem::path p(objfilename);
  string input_dir = boost::filesystem::path(objfilename).parent_path().c_str();

  ifstream input(objfilename.c_str());
  string line;
  while (getline(input, line)) {
    istringstream iss(line);
    string tok1;
    iss >> tok1;
    if(tok1 == "mtllib")
    {
      string tok2;

      iss >> tok2;
      string linemtl;

      ifstream inputmtl((input_dir + "/" + tok2).c_str());
      while (getline(inputmtl, linemtl))
      {
        istringstream issmtl(linemtl);
        issmtl >> tok1;
        if (tok1 == "map_Kd")
        {
          issmtl >> tok2;
          return input_dir  + "/" + tok2;
        }
      }
    }
  }
  return "";
}
#endif //OPENDETECTION_UTILS_H
