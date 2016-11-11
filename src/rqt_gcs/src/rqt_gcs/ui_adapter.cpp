#include "rqt_gcs/ui_adapter.h"

namespace rqt_gcs 
{   
    
    UIAdapter::UIAdapter(){ }
    
    UIAdapter::~UIAdapter(){ }
    
    UIAdapter * UIAdapter::Instance()
    {
        if(!instance)
            instance = new UIAdapter();
        
        return instance;
    }
    
    UIAdapter *UIAdapter::instance = nullptr;
}
    

