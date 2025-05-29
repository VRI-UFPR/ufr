/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Felipe Bombardelli
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
 */
	
// ============================================================================
//  Header
// ============================================================================

#include <cairo.h>
#include <stdio.h>
#include <gtk/gtk.h>
#include <pthread.h>
#include <ufr.h>
#include <time.h>

#define MAX_POINTS 100

GtkApplication* g_app;
pthread_t g_thread;

GtkWidget* g_box;
volatile int g_started;


// Estrutura para armazenar os pontos do gráfico
typedef struct {
    GtkWidget *drawing_area;
    double max_x, max_y;
    double x[MAX_POINTS];
    double y[MAX_POINTS];
    uint16_t count;             // contador de cada linha
    uint8_t index;               // indice de qual linha escrever
    double val[8];
} chart_t;

// ============================================================================
//  Private Functions
// ============================================================================

static
void chart_clean(chart_t* chart) {
    chart->index = 0;
}

static
void chart_put(chart_t* chart, double val_x, double val_y) {
    // printf("line %d %f\n", line, val_y);

    if (chart->count >= MAX_POINTS) {
        for (int j = 0; j < MAX_POINTS - 1; ++j) {
            chart->x[j] = chart->x[j + 1];
            chart->y[j] = chart->y[j + 1];
        }
        chart->x[MAX_POINTS - 1] = val_x;
        chart->y[MAX_POINTS - 1] = val_y;
    } else {
        const int idx = chart->count;
        chart->x[idx] = val_x;
        chart->y[idx] = val_y;
        chart->count += 1;
    }

    /*const double abs_val_y = fabs(val_y);
    if ( abs_val_y > chart->max_y ) {
        chart->max_y = abs_val_y;
    }*/

    // chart->line += 1;
}


static 
void gtk_cb_onClick (GtkWidget* widget, gpointer data) {
    // g_print ("Hello World\n");
    // int len = strlen(data);
    // ufr_write(&g_pipe_event, data, len);
}

// Função de desenho (callback para o sinal "draw" da GtkDrawingArea)
static void drawing_area_draw_func(GtkDrawingArea *area, cairo_t *cr, int width, int height, gpointer user_data) {
    // Limpar o fundo
    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0); // Branco
    cairo_paint(cr);

    // Desenhar os eixos (opcional)
    cairo_set_source_rgb(cr, 0.0, 0.0, 0.0); // Preto
    cairo_set_line_width(cr, 1.0);
    cairo_move_to(cr, 0, height / 2);
    cairo_line_to(cr, width, height / 2);
    cairo_stroke(cr);

    cairo_move_to(cr, width / 2, 0);
    cairo_line_to(cr, width / 2, height);
    cairo_stroke(cr);

    // Desenhar o gráfico
    const chart_t* chart = (chart_t*) user_data;
    if ( chart == NULL ) {
        return;
    }

    // Cores das linhas
    float colors[8][3] = {
        {0.0, 0.0, 1.0}, // azul
        {0.0, 1.0, 0.0}, // red
        {1.0, 0.0, 0.0}, // azul
        {0.0, 0.0, 1.0}, // azul
        {0.0, 0.0, 1.0}, // azul
        {0.0, 0.0, 1.0}, // azul
        {0.0, 0.0, 1.0}, // azul
        {0.0, 0.0, 1.0}, // azul
    };

    // 
    // for (uint8_t line=0; line<8; line++) {
        const uint16_t chart_count = chart->count;
        if (chart_count == 0) {
            // break;
        }

        // otimiza o acesso das variaveis
        const float* color = &colors[0];

        // Desenha a linha
        {
            cairo_set_source_rgb(cr, color[0], color[1], color[2]); // Azul
            cairo_set_line_width(cr, 2.0);

            // Mapear os dados para as coordenadas da área de desenho
            // Assumindo que os dados Y variam de -1.0 a 1.0 (seno/cosseno)
            // e X varia de 0 a MAX_POINTS-1
            double x_scale = (double)width / (MAX_POINTS - 1);
            double y_scale = (double)height / 2.0;
            double y_offset = height / 2.0;
            double x_offset = width / 2.0;

            // Set o cursor no primeiro ponto
            const double val_x0 = x_offset + chart->x[0] * x_scale;
            const double val_y0 = y_offset - (chart->y[0]/chart->max_y) * y_scale;
            cairo_move_to(cr, val_x0, val_y0);

            // Desenha os pontos
            for (uint16_t i = 1; i < chart_count; i++) {
                const double val_x = x_offset + chart->x[i] * x_scale;
                const double val_y = y_offset - (chart->y[i]/chart->max_y) * y_scale;
                cairo_line_to(cr, val_x, val_y);
            }

            //
            cairo_stroke(cr);
        }
    // }

    
}

static 
void gtk_cb_onActivate(GtkApplication* app, gpointer user_data) {
    GtkWidget* window = gtk_application_window_new (app);
    gtk_window_set_title (GTK_WINDOW (window), "Window");
    gtk_window_set_default_size (GTK_WINDOW (window), 640, 480);
    gtk_window_present (GTK_WINDOW (window));

    /*g_box = gtk_box_new (GTK_ORIENTATION_VERTICAL, 0);
    gtk_widget_set_halign (g_box, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (g_box, GTK_ALIGN_CENTER);
    gtk_window_set_child (GTK_WINDOW(window), g_box);*/

    // Criar a área de desenho
    GtkWidget* drawing_area = gtk_drawing_area_new();
    gtk_widget_set_hexpand(drawing_area, TRUE);
    gtk_widget_set_vexpand(drawing_area, TRUE);

    // Definir a função de desenho para a GtkDrawingArea no GTK4
    gtk_drawing_area_set_draw_func(GTK_DRAWING_AREA(drawing_area), drawing_area_draw_func, user_data, NULL);

    // Adicionar a área de desenho à janela
    gtk_window_set_child(GTK_WINDOW(window), drawing_area);

    // mark started
    g_started = 1;

    // update the link
    chart_t* chart = (chart_t*) user_data;
    chart->drawing_area = drawing_area;
}

static 
void* main_gtk(void* p_link) {
    link_t* link = (link_t*) p_link;

    // initialize the client/server pipe
    srand( time(NULL) );
    int app_id = random();
    char app_name[512];
    snprintf(app_name, 512, "org.gtk.chart_%d", app_id);
    printf("%s\n", app_name);
    g_app = gtk_application_new (app_name, G_APPLICATION_FLAGS_NONE);
    g_signal_connect (g_app, "activate", G_CALLBACK (gtk_cb_onActivate), link->gtw_obj);
    int status = g_application_run (G_APPLICATION (g_app), 0, NULL);
    g_object_unref (g_app);
    g_thread = 0;
    ufr_loop_set_end();

    // ufr_write(&g_pipe_event, "end", 4);
}


// ============================================================================
//  GTK Client 
// ============================================================================

static
int gtw_gtk_type(const link_t* link) {
	return 0;
}

static
int gtw_gtk_state(const link_t* link){
	return 0;
}

static
size_t gtw_gtk_size(const link_t* link, int type){
	return 0;
}

static
int gtw_gtk_boot(link_t* link, const ufr_args_t* args) {
    chart_t* chart = malloc(sizeof(chart_t));
    if ( chart == NULL ) {
        return ufr_error(link, -1, "sem memoria");
    }

    chart->count = 0;
    chart_clean(chart);
    chart->max_x = ufr_args_getf(args, "@max_x", 100.0);
    chart->max_y = ufr_args_getf(args, "@max", 1.0);

    // success
    link->gtw_obj = chart;
    return UFR_OK;
}

static
int gtw_gtk_start(link_t* link, int type, const ufr_args_t* args) {
    g_started = 0;
    pthread_create(&g_thread, NULL, main_gtk, link);
    
    // wait for activation function
    while ( g_started == 0 );
    return UFR_OK;
}

static
void gtw_gtk_stop(link_t* link, int type) {
    // if ( g_thread != 0 ) {
        // pthread_join(g_thread, NULL); 
    // }
}

static
int gtw_gtk_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t gtw_gtk_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

static
size_t gtw_gtk_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
int gtw_gtk_recv(link_t* link) {
    return -1;
}

static
ufr_gtw_api_t gtw_gtk = {
    .name = "chart",
	.type = gtw_gtk_type,
	.state = gtw_gtk_state,
	.size = gtw_gtk_size,
	.boot = gtw_gtk_boot,
	.start = gtw_gtk_start,
	.stop = gtw_gtk_stop,
	.copy = gtw_gtk_copy,
	.read = gtw_gtk_read,
	.write = gtw_gtk_write,
    .recv = gtw_gtk_recv,
};

// ============================================================================
//  Default Encoder
// ============================================================================

static
int ufr_enc_chart_boot(link_t* link, const ufr_args_t* args) {

    return UFR_OK;
}

static
void ufr_enc_chart_close(link_t* link) {
    
}

static
void ufr_enc_chart_clear(link_t* link) {

}

static
int ufr_enc_chart_put_u32(link_t* link, const uint32_t* val, int nitems) {
    int wrote = 0;
    
    return wrote;
}

static
int ufr_enc_chart_put_i32(link_t* link, const int32_t* val, int nitems) {
    chart_t* chart = (chart_t*) link->gtw_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        chart->val[ chart->index ] = (double) val[wrote];
        chart->index += 1;
    }
    return wrote;
}

static
int ufr_enc_chart_put_f32(link_t* link, const float* val, int nitems) {
    chart_t* chart = (chart_t*) link->gtw_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        chart->val[ chart->index ] = (double) val[wrote];
        chart->index += 1;
    }
    return wrote;
}

static
int ufr_enc_chart_put_u64(link_t* link, const uint64_t* val, int nitems) {
    int wrote = 0;
    
    return wrote;
}

static
int ufr_enc_chart_put_i64(link_t* link, const int64_t* val, int nitems) {
    int wrote = 0;
    
    return wrote;
}

static
int ufr_enc_chart_put_f64(link_t* link, const double* val, int nitems) {
    int wrote = 0;
    
    return wrote;
}


static
int ufr_enc_chart_put_str(link_t* link, const char* val) {
    
    return 0;
}

static
int ufr_enc_chart_put_cmd(link_t* link, char cmd) {
    chart_t* chart = (chart_t*) link->gtw_obj;
    if ( cmd == '\n' || cmd == (char) EOF ) {
        chart_put(chart, chart->val[0], chart->val[1]);
        gtk_widget_queue_draw(chart->drawing_area);
        chart_clean(chart);
    } else {
        return ufr_error(link, 1, "Command %d not found", cmd);
    }

    return UFR_OK;
}

static
int ufr_enc_chart_put_raw(link_t* link, const uint8_t* buffer, int size) {
    return size;
}

int ufr_enc_chart_enter(link_t* link, size_t maxsize) {
    return UFR_OK;
}


int ufr_enc_chart_leave(link_t* link) {
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_chart2d_api = {
    .boot = ufr_enc_chart_boot,
    .close = ufr_enc_chart_close,
    .clear = ufr_enc_chart_clear,

    .put_u32 = ufr_enc_chart_put_u32,
    .put_i32 = ufr_enc_chart_put_i32,
    .put_f32 = ufr_enc_chart_put_f32,

    .put_u64 = ufr_enc_chart_put_u64,
    .put_i64 = ufr_enc_chart_put_i64,
    .put_f64 = ufr_enc_chart_put_f64,

    .put_cmd = ufr_enc_chart_put_cmd,
    .put_str = ufr_enc_chart_put_str,
    .put_raw = ufr_enc_chart_put_raw,

    .enter = ufr_enc_chart_enter,
    .leave = ufr_enc_chart_leave
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_chart2d_new(link_t* link, int type) {
    link->gtw_api = &gtw_gtk;
    link->enc_api = &ufr_enc_chart2d_api;
	return UFR_OK;
}


