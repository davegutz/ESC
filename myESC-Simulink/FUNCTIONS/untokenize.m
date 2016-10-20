function [result] = untokenize(ctokens, tokens)
% function [result] = untokenize(ctokens, tokens)
% reconstruct result of tokenize; useful for after changing tokens or
% ctokens
%% GE PROPRIETARY INFORMATION:
% The information contained in this document is GE proprietary
% information and is disclosed in confidence.  It is the property of GE and
% shall not be used, disclosed to others, reproduced, or
% exported without the express written consent of GE, including, but
% without limitation, it is not to be used in the creation, manufacture,
% development, or derivation of any repairs, modifications, spare parts, designs,
% or configuration changes
% or to obtain FAA or any other government or regulatory approval to do so.
% If consent is given for reproduction in whole or in part, this notice and t
% he notice set forth on each page of this document shall appear in
% any such reproduction in whole or in part.  
result  = [];
nTok    = length(tokens);
nC_Tok  = length(ctokens);
if nC_Tok-nTok > 1 || nC_Tok<nTok,
    fprintf(1, 'WARNING(untokenize):  bad correspondence of ctokens and tokens');
end

% Reconstruct
for i=1:nC_Tok,
    result = sprintf('%s%s', result, ctokens{i});
    if nTok>=i, result = sprintf('%s%s', result, tokens{i});end
end

