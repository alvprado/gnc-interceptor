include(GNUInstallDirs)

function(gnc_install_library target export_name)
    set_target_properties(${target} PROPERTIES EXPORT_NAME "${export_name}")
    get_target_property(target_type ${target} TYPE)
    if(target_type STREQUAL "INTERFACE_LIBRARY")
        set(include_scope INTERFACE)
    else()
        set(include_scope PUBLIC)
    endif()

    target_include_directories(${target} ${include_scope}
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/gnc_core>)
    get_target_property(source_dir ${target} SOURCE_DIR)
    install(DIRECTORY "${source_dir}/include/"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnc_core")
    install(TARGETS ${target} EXPORT gnc_coreTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endfunction()
