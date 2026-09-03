function printComparison(tableTitle, names, xTrue, xRetrieved)
%PRINTCOMPARISON Print true, retrieved, and error values.

fprintf('\n%s:\n', tableTitle);
fprintf('%12s    %12s    %12s    %12s\n', ...
    'Quantity', 'True', 'Retrieved', 'Error');
for i = 1:numel(xTrue)
    fprintf('%12s    %12.6f    %12.6f    %12.3e\n', ...
        names{i}, xTrue(i), xRetrieved(i), xRetrieved(i) - xTrue(i));
end

end