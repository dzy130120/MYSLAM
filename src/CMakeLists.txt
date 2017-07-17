ADD_LIBRARY(myslam SHARED MapPoint.cpp Map.cpp Camera.cpp Config.cpp Frame.cpp FeatureOperater.cpp VisualOdometry.cpp )
target_link_libraries(myslam ${THIRD_PARTY_LIBS})

