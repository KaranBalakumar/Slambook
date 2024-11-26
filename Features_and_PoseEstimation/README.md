# Further Updates Needed
- We found that the ORB feature points provided by OpenCV are not evenly distributed in the image. Propose a way to make the distribution of feature points more evenly? - maybe Adaptive Non-Maximal suppression???
- In the feature point matching, mismatches will inevitably be encountered.
What happens if we put the wrong match into PnP or ICP? What methods could be used to avoid mismatches?
- Use the SE3 class in Sophus to write the nodes and edges of g2o and implement the optimization of PnP and ICP
- Implement the optimization of PnP and ICP in Ceres
- Investigate why FLANN can quickly handle matching problems. In addition to FLANN, what other ways to accelerate matching?