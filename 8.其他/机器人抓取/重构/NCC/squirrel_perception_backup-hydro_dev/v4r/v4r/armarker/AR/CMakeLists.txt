PROJECT(AR)

set (AR_C 
arDetectMarker2.c  
mDisp.c            
paramDistortion.c
arDetectMarker.c     
mDup.c             
paramFile.c
mFree.c            
paramGet.c
arGetCode.c          
mInv.c             
vAlloc.c
arGetMarkerInfo.c   
mMul.c             
vDisp.c
arGetTransMat2.c     
mAlloc.c       
mPCA.c             
vFree.c
arGetTransMat3.c     
mAllocDup.c    
mSelfInv.c         
vHouse.c
arGetTransMat.c      
mAllocInv.c    
mTrans.c           
vInnerP.c
arGetTransMatCont.c  
mAllocMul.c    
mUnit.c            
vTridiag.c  
mAllocTrans.c  
paramChangeSize.c
arLabeling.c         
mAllocUnit.c   
paramDecomp.c
arUtil.c             
mDet.c         
paramDisp.c)

set (AR_H  
config.h     
ar.h                 
)
include_directories(../)
add_library(ar STATIC ${AR_C} ${AR_H})

