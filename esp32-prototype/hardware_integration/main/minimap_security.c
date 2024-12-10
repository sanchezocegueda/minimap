#include "psa/crypto.h"

psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
psa_key_id_t key_id;

void import_key(const uint8_t *key, size_t key_len)
{
    psa_status_t status;
    printf("Import an AES key...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();
    if (status != PSA_SUCCESS) {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    /* Set key attributes */
    psa_set_key_usage_flags(&attributes, 0);
    psa_set_key_algorithm(&attributes, 0);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);

    /* Import the key */
    status = psa_import_key(&attributes, key, key_len, &key_id);
    if (status != PSA_SUCCESS) {
        printf("Failed to import key\n");
        return;
    }
    printf("Imported a key\n");
}

void free_key(psa_key_id_t key_id, psa_key_attributes_t *attributes)
{
    /* Free the attributes */
    psa_reset_key_attributes(attributes);

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();
}