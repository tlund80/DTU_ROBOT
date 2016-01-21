#ifndef SCENEDATA_H
#define SCENEDATA_H
#include "viewdata.h"

class SceneData
{
public:   
   SceneData();

   inline void setName(std::string name){name_ = name;}
   inline std::string getName(void){return name_;}
   std::vector<boost::shared_ptr<ViewData> > data;

private:

   std::string name_;





};

#endif // SCENEDATA_H
