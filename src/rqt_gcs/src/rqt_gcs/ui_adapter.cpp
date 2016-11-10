#include "rqt_gcs/ui_adapter.h"

namespace rqt_gcs 
{
    UIAdapter * UIAdapter::instance = nullptr;
    
    UIAdapter::UIAdapter(){    
    }
    
    UIAdapter::~UIAdapter(){
    }
    
    UIAdapter * UIAdapter::Instance()
    {
        if(!instance)
            instance = new UIAdapter();
        
        return instance;
    }
}
    

