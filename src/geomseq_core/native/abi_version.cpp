// abi_version.cpp
// The version of this library's binary interface. Both wrappers read it at
// load time and refuse a binary whose number differs from their own.
//
// Bump it when an exported signature changes, or an export is added or
// removed, together with its two copies:
//   ABI_VERSION  in src/geomseq_core/native_bridge.py
//   AbiVersion   in src/gha/Native/NativeLibraryLoader.cs

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

// 2: point_in_curve added.
// 3: point_in_curve lost its out_projected argument. The caller already holds
//    that point -- it projects into the plane before calling.
// 4: point_in_curves added.
// 5: coverage_hatch_region, coverage_plan and the coverage_result_*
//    functions added.
// 6: the coverage exports' link accepts ONE_WAY_SEAMS (2). The signatures
//    did not change; a version-5 binary would refuse 2 with its link error.
// 7: resample_on_lattice added.
DLL_EXPORT int geomseq_abi_version() {
    return 7;
}

} // extern "C"
