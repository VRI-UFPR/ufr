/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Visao Robotica Imagem (VRI)
 *   Felipe Bombardelli
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */

// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <telebot.h>
#include <ufr.h>

typedef struct {
    telebot_handler_t handle;
    telebot_user_t me;
    telebot_message_t message;
    telebot_update_t *updates;
    int req_count;
    int req_index;
} telegram_obj_t;

// ============================================================================
//  Decoder
// ============================================================================


// ============================================================================
//  Encoder
// ============================================================================


// ============================================================================
//  Gateway Socket
// ============================================================================

static
int ufr_gtw_telegram_type(const link_t* link) {
    return 0;
}

static
int ufr_gtw_telegram_state(const link_t* link) {
    return 0;
}

static
size_t ufr_gtw_telegram_size(const link_t* link, int type) {
    return 0;
}

static
int ufr_gtw_telegram_boot (link_t* link, const ufr_args_t* args) {
    
    telegram_obj_t* obj = malloc(sizeof(telegram_obj_t));
    if ( obj == NULL ) {
        return -1;
    }
    obj->req_index = 0;
    obj->req_count = -1;

    char* token = "aaaa";

    if (telebot_create(&obj->handle, token) != TELEBOT_ERROR_NONE) {
        printf("Telebot create failed\n");
        return -1;
    }

    telebot_user_t me;
    if (telebot_get_me(obj->handle, &obj->me) != TELEBOT_ERROR_NONE) {
        printf("Failed to get bot information\n");
        telebot_destroy(obj->handle);
        return -1;
    }

    printf("ID: %d\n", me.id);
    printf("First Name: %s\n", me.first_name);
    printf("User Name: %s\n", me.username);
    telebot_put_me(&me);

    return UFR_OK;
}

static
int ufr_gtw_telegram_start (link_t* link, int type, const ufr_args_t* args) {
    telegram_obj_t* obj = link->gtw_obj;

    if ( type == UFR_START_SERVER ) {


    } else if ( type == UFR_START_CLIENT ) {
        return -1;
    } else {
        return -1;
    }

    // success
    return UFR_OK;
}

static
void ufr_gtw_telegram_stop(link_t* link, int type) {
    telegram_obj_t* obj = link->gtw_obj;
    telebot_destroy(obj->handle);
    free(obj);
}

static
int ufr_gtw_telegram_recv(link_t* link) {
    telegram_obj_t* obj = link->gtw_obj;

    // Still there are messages in the buffer
    if ( obj->req_index <= obj->req_count ) {
        obj->message = obj->updates[obj->req_index].message;
        return UFR_OK;
    }

    // Get new messages        
    int offset = -1;   
    telebot_update_type_e update_types[] = {TELEBOT_UPDATE_TYPE_MESSAGE};
    const telebot_error_e ret = telebot_get_updates(obj->handle, offset, 20, 0, update_types, 0, &obj->updates, &obj->req_count);
    if (ret != TELEBOT_ERROR_NONE) {
        return -1;
    }

    // Success
    obj->req_index = 0;
    ufr_info(link, "Number of updates: %d", obj->req_count);
    return UFR_OK;
}

static
int ufr_gtw_telegram_recv_async(link_t* link) {
    telegram_obj_t* obj = link->gtw_obj;

    // No message
    return -1;
}

static
size_t ufr_gtw_telegram_read(link_t* link, char* buffer, size_t max_size) {
    return 0;
}

static
size_t ufr_gtw_telegram_write(link_t* link, const char* buffer, size_t size) {
    telegram_obj_t* obj = link->gtw_obj;

    const telebot_error_e ret = telebot_send_message(obj->handle, obj->message.chat->id, buffer, "HTML", false, false, 0, "");
    if (ret != TELEBOT_ERROR_NONE) {
        return ret;
    }

    return 0;
}

static
int ufr_gtw_telegram_ready(link_t* link) {
    return UFR_OK;
}

static
ufr_gtw_api_t ufr_gtw_telegram_api = {
    .name = "telegram/server",
    .type = ufr_gtw_telegram_type,
    .state = ufr_gtw_telegram_state,
    .size = ufr_gtw_telegram_size,
    .boot = ufr_gtw_telegram_boot,
    .start = ufr_gtw_telegram_start,
    .stop = ufr_gtw_telegram_stop,
    .copy = NULL,
    .recv = ufr_gtw_telegram_recv,
    .recv_async = ufr_gtw_telegram_recv_async,
    .read = ufr_gtw_telegram_read,
    .write = ufr_gtw_telegram_write,
    .ready = ufr_gtw_telegram_ready,
};

// ============================================================================
//  Publico
// ============================================================================

int ufr_gtw_telegram_new(link_t* link, int type) {
    link->gtw_api = &ufr_gtw_telegram_api;
    return UFR_OK;
}


int aaaamain(int argc, char *argv[])
{
    /*
    printf("Welcome to Echobot\n");

    FILE *fp = fopen(".token", "r");
    if (fp == NULL)
    {
        printf("Failed to open .token file\n");
        return -1;
    }

    char token[1024];
    if (fscanf(fp, "%s", token) == 0)
    {
        printf("Failed to read token\n");
        fclose(fp);
        return -1;
    }
    printf("Token: %s\n", token);
    fclose(fp);

    telebot_handler_t handle;
    if (telebot_create(&handle, token) != TELEBOT_ERROR_NONE)
    {
        printf("Telebot create failed\n");
        return -1;
    }

    telebot_user_t me;
    if (telebot_get_me(handle, &me) != TELEBOT_ERROR_NONE)
    {
        printf("Failed to get bot information\n");
        telebot_destroy(handle);
        return -1;
    }

    printf("ID: %d\n", me.id);
    printf("First Name: %s\n", me.first_name);
    printf("User Name: %s\n", me.username);

    telebot_put_me(&me);
    

    int index, count, offset = -1;
    telebot_error_e ret;
    
    telebot_update_type_e update_types[] = {TELEBOT_UPDATE_TYPE_MESSAGE};

    while (1)
    {
        telebot_update_t *updates;
        ret = telebot_get_updates(handle, offset, 20, 0, update_types, 0, &updates, &count);
        if (ret != TELEBOT_ERROR_NONE)
            continue;
        printf("Number of updates: %d\n", count);
        for (index = 0; index < count; index++)
        {
            message = updates[index].message;
            if (message.text)
            {
                printf("%s: %s \n", message.from->first_name, message.text);
                if (strstr(message.text, "/dice"))
                {
                    telebot_send_dice(handle, message.chat->id, false, 0, "");
                }
                else
                {
                    char str[4096];
                    if (strstr(message.text, "/start"))
                    {
                        snprintf(str, SIZE_OF_ARRAY(str), "Hello %s", message.from->first_name);
                    }
                    else
                    {
                        snprintf(str, SIZE_OF_ARRAY(str), "<i>%s</i>", message.text);
                    }
                    // ret = telebot_send_message(handle, message.chat->id, str, "HTML", false, false, updates[index].message.message_id, "");
                    ret = telebot_send_message(handle, message.chat->id, str, "HTML", false, false, 0, "");

                    telebot_send_photo(handle, message.chat->id, "foto.jpg", true, "opa", "HTML", false, 0, "");
                }
                if (ret != TELEBOT_ERROR_NONE)
                {
                    printf("Failed to send message: %d \n", ret);
                }
            }
            offset = updates[index].update_id + 1; // FALTA ISSO
        }
        telebot_put_updates(updates, count);  // FALTA ISSO

        sleep(1);
    }

    telebot_destroy(handle);*/

    return 0;
}







