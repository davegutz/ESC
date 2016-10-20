function SortedStruct = OrderAllFields(StructToSort)
% function SortedStruct = OrderAllFields(StructToSort)
%
% Sorts all fieldnames in a structure array alphabetically
% on all levels.
% Recursive function.
%
if ~isstruct(StructToSort)
  SortedStruct = StructToSort;
  if iscell(SortedStruct)
    for iCell = 1:numel(SortedStruct)
      SortedStruct{iCell} = OrderAllFields(SortedStruct{iCell});
    end
  end
else
  SortedStruct = orderfields(StructToSort);
  SortedFields = fieldnames(SortedStruct);
  for iField = 1:length(SortedFields)
    SortedStruct.(SortedFields{iField}) = ...
      OrderAllFields(SortedStruct.(SortedFields{iField}));
  end
end

return
