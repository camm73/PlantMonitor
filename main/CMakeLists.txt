idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "")

target_add_binary_data(${COMPONENT_TARGET} "certs/root-ca.pem" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/certificate.pem.crt" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/private.pem.key" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/public.pem.key" TEXT)