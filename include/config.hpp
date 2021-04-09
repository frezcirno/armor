#pragma once

#include "toml.h"
#include "debug.h"
#include <string>

namespace armor {

/**
 * 配置文件结构体
 */
struct Config {
    toml::Value config;

    explicit Config(const std::string &file_path) {
        /* parse config.toml */
        std::ifstream ifs(file_path);
        toml::ParseResult pr = toml::parse(ifs);
        ifs.close();
        if (!pr.valid()) {
            PRINT_ERROR("[config] config toml error: %s\n", pr.errorReason.c_str());
            PRINT_ERROR("[config] abort!\n");
            exit(0);
        }
        config = pr.value;
    }

    template <typename T>
    inline typename toml::call_traits<T>::return_type get(const std::string &key) const {
        return config.get<T>(key);
    }
};

}  // namespace armor