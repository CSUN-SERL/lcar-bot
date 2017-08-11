#include <gcs/qt/ui_adapter.h>

namespace gcs 
{   
    UIAdapter *UIAdapter::instance = nullptr;
    
    UIAdapter::UIAdapter(){ }
    
    UIAdapter::~UIAdapter(){ }
    
    UIAdapter * UIAdapter::Instance()
    {
        if(!instance)
            instance = new UIAdapter();
        
        return instance;
    }    
}
    