#include <iostream>
#include <dlfcn.h>
#include <stdexcept>

// Define function pointers for the signatures we want to test
typedef void (*ConnectFuncANSI)(const char*, const char*, const char*);
typedef void (*DisconnectFunc)();

int main() {
    std::cout << "Loading library..." << std::endl;
    // Load explicitly to debug
    void* handle = dlopen("./libhla_plugin.so", RTLD_LAZY);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << std::endl;
        return 1;
    }

    // Reset errors
    dlerror();

    // Load symbols
    void* connectSym = dlsym(handle, "Connect");
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol 'Connect': " << dlsym_error << std::endl;
        dlclose(handle);
        return 1;
    }

    void* disconnectSym = dlsym(handle, "Disconnect");
    DisconnectFunc Disconnect = (DisconnectFunc)disconnectSym;

    std::cout << "Symbol 'Connect' found. Attempting to call..." << std::endl;

    try {
        std::cout << "--- Test 1: Calling with char* (ANSI) ---" << std::endl;
        ConnectFuncANSI ConnectA = (ConnectFuncANSI)connectSym;
        
        // Use a unique name to avoid "Federation Execution Already Exists" error
        // Use the absolute path confirmed earlier
        ConnectA("TestFederation_ANSI_Verify", "TestFederate", "/home/vicen/ISDEFE/Pruebas Unity/Assets/StreamingAssets/BoxFOM.xml"); 
        std::cout << "--- Test 1 Success: returned without crash. ---" << std::endl;
        
        if (Disconnect) Disconnect();
    } catch (const std::exception& e) {
        std::cout << "--- Test 1 Failed: std::exception: " << e.what() << " ---" << std::endl;
    } catch (...) {
        std::cout << "--- Test 1 Failed: Exception thrown (Check FOM path and Federation Name). ---" << std::endl;
    }

    dlclose(handle);
    return 0;
}
