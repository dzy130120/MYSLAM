ADD_LIBRARY(myslam SHARED Camera.cpp Config.cpp Frame.cpp FeatureOperater.cpp VisualOdometry.cpp MapPoint.cpp)
target_link_libraries(myslam ${THIRD_PARTY_LIBS})

