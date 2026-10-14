function updateTrainTransform(train)
% UPDATETRAINTRANSFORM - Applies train.x/y/z to its hgtransform.
    T = makehgtform('translate', [train.x train.y train.z]);
    set(train.mainTransform, 'Matrix', T);
end
