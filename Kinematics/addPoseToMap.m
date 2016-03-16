function Rob = addPoseToMap(Rob,Tim)
%Update the pose vectors

    Rob.state.x_full(:,Tim.currentFrame) = Rob.state.x;
    Rob.state.dr_full(:,Tim.currentFrame) = Rob.state.dr;
    Rob.state.gt_full(:,Tim.currentFrame) = Rob.state.gt;
end