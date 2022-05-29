function(empc_test testname)
    add_executable(${testname}
        ${testname}.cpp
    )
    target_link_libraries(${testname}
        PRIVATE
        fmt::fmt
        empc 
        gtest_main
    )
    gtest_discover_tests(${testname})
endfunction()

empc_test(problem_test)
empc_test(memory_test)
empc_test(osqpsolver_test)
