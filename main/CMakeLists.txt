idf_component_register(SRCS "sketch.ino.cpp" "CommandHandler.cpp"
                    INCLUDE_DIRS "."
		    REQUIRES mbedtls driver bt arduino wpa_supplicant)
