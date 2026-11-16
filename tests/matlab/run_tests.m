function results = run_tests(names)
%RUN_TESTS Run the MATLAB tests and write a summary to tests/matlab/results.txt.
%   run_tests            all tests (takes several minutes: IK + 5000-pose collision search)
%   run_tests(["testServoMapping" ...])   only the named tests
arguments
    names (1,:) string = string.empty
end
here = fileparts(mfilename("fullpath"));
logFile = fullfile(here, "results.txt");
if isfile(logFile), delete(logFile); end
setenv("ARM_TEST_LOG", logFile);
c = onCleanup(@() setenv("ARM_TEST_LOG", ""));

t0 = tic;
if isempty(names)
    results = runtests(fullfile(here, "test_arm.m"));
else
    results = runtests(fullfile(here, "test_arm.m"), "ProcedureName", names);
end

fid = fopen(logFile, "a");
fprintf(fid, "\n%-45s %-6s %8s\n", "Test", "Result", "Time(s)");
for r = results(:)'
    status = "PASS"; if r.Failed, status = "FAIL"; elseif r.Incomplete, status = "INCOMPLETE"; end
    fprintf(fid, "%-45s %-6s %8.1f\n", erase(r.Name, "test_arm/"), status, r.Duration);
end
fprintf(fid, "\n%d/%d passed in %.0f s\n", sum([results.Passed]), numel(results), toc(t0));
fclose(fid);
end
