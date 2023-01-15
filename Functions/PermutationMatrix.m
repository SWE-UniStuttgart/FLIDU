
%% Permutes input variations
%% Code:
function [nVariation, nPermutation, Permutation] = PermutationMatrix(Variation)

if ~isempty(Variation)
    [nVariation,~]            = size(Variation);
    
    for iVariation                  = 1:nVariation
        VariationDepth(iVariation)  = length(Variation{iVariation,2});
    end
    nPermutation                    = prod(VariationDepth);
    
    Permutation     = [];
    for iVariation  = 1:nVariation
        
        sizeP   = size(Permutation);
        P_resh        = reshape(repmat(Permutation,1,VariationDepth(iVariation))',sizeP(2),sizeP(1)*VariationDepth(iVariation))';
        
        Permutation	= [P_resh ....
            repmat([1:VariationDepth(iVariation) ]',prod(VariationDepth(1:iVariation-1)),1)];
    end
    
else
    nVariation      = 0;
    nPermutation    = 1;
    Permutation     = NaN;
end
end