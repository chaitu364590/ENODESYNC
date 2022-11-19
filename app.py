#from asyncio import log
import cv2
import streamlit as st
import cv2.aruco as aruco
import numpy as np
st.title("ENODESYNC")
st.caption("AR FOR IOT")
st.markdown("---")
url = "http://192.168.137.171"
st.write("check out this [CAMERA CONFIGURATION](%s)" % url)





st.title("Live Feed")
id_marker=1
aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters=aruco.DetectorParameters_create()
#image_augment=cv2.imread("C:/Users/lenovo/POTHOLE/PIC_A_1.jpg")
video_augment=cv2.VideoCapture("C:/Users/lenovo/Downloads/macbook-pro.GIF") #hv
run = st.checkbox('Run',key="1")
#FRAME_WINDOW = st.image([])
#F1 = st.image([])
F2 = st.image([])







#F3 = st.image([])
#F4 = st.image([])
#http://192.168.29.220:81/stream
camera = cv2.VideoCapture(0)
detection = False
frame_count = 0

height_marker, width_marker = 100, 100
_, image_video = video_augment.read()
image_video = cv2.resize(image_video, (width_marker, height_marker))




#st.markdown("check out this [link](%s)" % url)

def augmentation(bbox, img, img_augment):
    top_left=bbox[0][0][0], bbox[0][0][1]
    top_right=bbox[0][1][0], bbox[0][1][1]
    bottom_right=bbox[0][2][0], bbox[0][2][1]
    bottom_left=bbox[0][3][0], bbox[0][3][1]
    
    height, width, _, =img_augment.shape
    points_1=np.array([top_left, top_right, bottom_right, bottom_left])
    points_2=np.float32([[0, 0], [width, 0],[width, height], [0, height]])
    matrix, _ =cv2.findHomography(points_2,points_1)
    image_out=cv2.warpPerspective(img_augment, matrix, (img.shape[1], img.shape[0]))
    cv2.fillConvexPoly(img, points_1.astype(int), (0,0,1))
    image_out=img+image_out
    return image_out
while run:
    _, frame = camera.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #F1.image(frame)
    if detection == False:
        video_augment.set(cv2.CAP_PROP_POS_FRAMES, 0)
        frame_count = 0
    else:
        if frame_count == video_augment.get(cv2.CAP_PROP_FRAME_COUNT):
            video_augment.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frame_count = 0
        _, image_video = video_augment.read()
        image_video = cv2.resize(image_video, (width_marker, height_marker))
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners, ids, rejected=aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
    if ids is not None and ids[0]==id_marker:
        detection = True
        #aruco.drawDetectedMarkers(frame, corners)
        frame=augmentation(np.array(corners)[0], frame, image_video)
    F2.image(frame)
    frame_count += 1
    
    #webbrowser.open('https://my.spline.design/3dtextbluecopy-86cab89ddbb0c385dbd3942a6b187088/')
    #webbrowser.open_new_tab("https://my.spline.design/purpleiconsset1copy-6c433258002d4b52f5860fcdd49b4aeb/")


code = '''#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static mtmn_config_t mtmn_config = {0};
static int8_t detection_enabled = 0;
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;
static face_id_list id_list = {0};

static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size){
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t * filter, int value){
    if(!filter->values){
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...){
    char loc_buf[64];
    char * temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if(len >= sizeof(loc_buf)){
        temp = (char*)malloc(len+1);
        if(temp == NULL) {
            return 0;
        }
    }
    vsnprintf(temp, len+1, format, arg);
    va_end(arg);
    rgb_print(image_matrix, color, temp);
    if(len > 64){
        free(temp);
    }
    return len;
}

static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id){
    int x, y, w, h, i;
    uint32_t color = FACE_COLOR_YELLOW;
    if(face_id < 0){
        color = FACE_COLOR_RED;
    } else if(face_id > 0){
        color = FACE_COLOR_GREEN;
    }
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    for (i = 0; i < boxes->len; i++){
        // rectangle box
        x = (int)boxes->box[i].box_p[0];
        y = (int)boxes->box[i].box_p[1];
        w = (int)boxes->box[i].box_p[2] - x + 1;
        h = (int)boxes->box[i].box_p[3] - y + 1;
        fb_gfx_drawFastHLine(&fb, x, y, w, color);
        fb_gfx_drawFastHLine(&fb, x, y+h-1, w, color);
        fb_gfx_drawFastVLine(&fb, x, y, h, color);
        fb_gfx_drawFastVLine(&fb, x+w-1, y, h, color);
#if 0
        // landmark
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)boxes->landmark[i].landmark_p[j];
            y0 = (int)boxes->landmark[i].landmark_p[j+1];
            fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
        }
#endif
    }
}

static int run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes){
    dl_matrix3du_t *aligned_face = NULL;
    int matched_id = 0;

    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        Serial.println("Could not allocate face recognition buffer");
        return matched_id;
    }
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){
        if (is_enrolling == 1){
            int8_t left_sample_face = enroll_face(&id_list, aligned_face);

            if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                Serial.printf("Enrolling Face ID: %d\n", id_list.tail);
            }
            Serial.printf("Enrolling Face ID: %d sample %d\n", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            if (left_sample_face == 0){
                is_enrolling = 0;
                Serial.printf("Enrolled Face ID: %d\n", id_list.tail);
            }
        } else {
            matched_id = recognize_face(&id_list, aligned_face);
            if (matched_id >= 0) {
                Serial.printf("Match Face ID: %u\n", matched_id);
                rgb_printf(image_matrix, FACE_COLOR_GREEN, "Hello Subject %u", matched_id);
            } else {
                Serial.println("No Match Found");
                rgb_print(image_matrix, FACE_COLOR_RED, "Intruder Alert!");
                matched_id = -1;
            }
        }
    } else {
        Serial.println("Face Not Aligned");
        //rgb_print(image_matrix, FACE_COLOR_YELLOW, "Human Detected");
    }

    dl_matrix3du_free(aligned_face);
    return matched_id;
}

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t out_len, out_width, out_height;
    uint8_t * out_buf;
    bool s;
    bool detected = false;
    int face_id = 0;
    if(!detection_enabled || fb->width > 400){
        size_t fb_len = 0;
        if(fb->format == PIXFORMAT_JPEG){
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        } else {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }
        esp_camera_fb_return(fb);
        int64_t fr_end = esp_timer_get_time();
        Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
        return res;
    }

    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
    if (!image_matrix) {
        esp_camera_fb_return(fb);
        Serial.println("dl_matrix3du_alloc failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    out_buf = image_matrix->item;
    out_len = fb->width * fb->height * 3;
    out_width = fb->width;
    out_height = fb->height;

    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    esp_camera_fb_return(fb);
    if(!s){
        dl_matrix3du_free(image_matrix);
        Serial.println("to rgb888 failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);

    if (net_boxes){
        detected = true;
        if(recognition_enabled){
            face_id = run_face_recognition(image_matrix, net_boxes);
        }
        draw_face_boxes(image_matrix, net_boxes, face_id);
        free(net_boxes->score);
        free(net_boxes->box);
        free(net_boxes->landmark);
        free(net_boxes);
    }

    jpg_chunking_t jchunk = {req, 0};
    s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
    dl_matrix3du_free(image_matrix);
    if(!s){
        Serial.println("JPEG compression failed");
        return ESP_FAIL;
    }

    int64_t fr_end = esp_timer_get_time();
    Serial.printf("FACE: %uB %ums %s%d\n", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start)/1000), detected?"DETECTED ":"", face_id);
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    dl_matrix3du_t *image_matrix = NULL;
    bool detected = false;
    int face_id = 0;
    int64_t fr_start = 0;
    int64_t fr_ready = 0;
    int64_t fr_face = 0;
    int64_t fr_recognize = 0;
    int64_t fr_encode = 0;

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while(true){
        detected = false;
        face_id = 0;
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            fr_start = esp_timer_get_time();
            fr_ready = fr_start;
            fr_face = fr_start;
            fr_encode = fr_start;
            fr_recognize = fr_start;
            if(!detection_enabled || fb->width > 400){
                if(fb->format != PIXFORMAT_JPEG){
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if(!jpeg_converted){
                        Serial.println("JPEG compression failed");
                        res = ESP_FAIL;
                    }
                } else {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            } else {

                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

                if (!image_matrix) {
                    Serial.println("dl_matrix3du_alloc failed");
                    res = ESP_FAIL;
                } else {
                    if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){
                        Serial.println("fmt2rgb888 failed");
                        res = ESP_FAIL;
                    } else {
                        fr_ready = esp_timer_get_time();
                        box_array_t *net_boxes = NULL;
                        if(detection_enabled){
                            net_boxes = face_detect(image_matrix, &mtmn_config);
                        }
                        fr_face = esp_timer_get_time();
                        fr_recognize = fr_face;
                        if (net_boxes || fb->format != PIXFORMAT_JPEG){
                            if(net_boxes){
                                detected = true;
                                if(recognition_enabled){
                                    face_id = run_face_recognition(image_matrix, net_boxes);
                                }
                                fr_recognize = esp_timer_get_time();
                                draw_face_boxes(image_matrix, net_boxes, face_id);
                                free(net_boxes->score);
                                free(net_boxes->box);
                                free(net_boxes->landmark);
                                free(net_boxes);
                            }
                            if(!fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)){
                                Serial.println("fmt2jpg failed");
                                res = ESP_FAIL;
                            }
                            esp_camera_fb_return(fb);
                            fb = NULL;
                        } else {
                            _jpg_buf = fb->buf;
                            _jpg_buf_len = fb->len;
                        }
                        fr_encode = esp_timer_get_time();
                    }
                    dl_matrix3du_free(image_matrix);
                }
            }
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();

        int64_t ready_time = (fr_ready - fr_start)/1000;
        int64_t face_time = (fr_face - fr_ready)/1000;
        int64_t recognize_time = (fr_recognize - fr_face)/1000;
        int64_t encode_time = (fr_encode - fr_recognize)/1000;
        int64_t process_time = (fr_encode - fr_start)/1000;
        
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
        Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
            (uint32_t)(_jpg_buf_len),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
            avg_frame_time, 1000.0 / avg_frame_time,
            (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
            (detected)?"DETECTED ":"", face_id
        );
    }

    last_frame = 0;
    return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;

    if(!strcmp(variable, "framesize")) {
        if(s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    }
    else if(!strcmp(variable, "quality")) res = s->set_quality(s, val);
    else if(!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
    else if(!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
    else if(!strcmp(variable, "saturation")) res = s->set_saturation(s, val);
    else if(!strcmp(variable, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)val);
    else if(!strcmp(variable, "colorbar")) res = s->set_colorbar(s, val);
    else if(!strcmp(variable, "awb")) res = s->set_whitebal(s, val);
    else if(!strcmp(variable, "agc")) res = s->set_gain_ctrl(s, val);
    else if(!strcmp(variable, "aec")) res = s->set_exposure_ctrl(s, val);
    else if(!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
    else if(!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
    else if(!strcmp(variable, "awb_gain")) res = s->set_awb_gain(s, val);
    else if(!strcmp(variable, "agc_gain")) res = s->set_agc_gain(s, val);
    else if(!strcmp(variable, "aec_value")) res = s->set_aec_value(s, val);
    else if(!strcmp(variable, "aec2")) res = s->set_aec2(s, val);
    else if(!strcmp(variable, "dcw")) res = s->set_dcw(s, val);
    else if(!strcmp(variable, "bpc")) res = s->set_bpc(s, val);
    else if(!strcmp(variable, "wpc")) res = s->set_wpc(s, val);
    else if(!strcmp(variable, "raw_gma")) res = s->set_raw_gma(s, val);
    else if(!strcmp(variable, "lenc")) res = s->set_lenc(s, val);
    else if(!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
    else if(!strcmp(variable, "wb_mode")) res = s->set_wb_mode(s, val);
    else if(!strcmp(variable, "ae_level")) res = s->set_ae_level(s, val);
    else if(!strcmp(variable, "face_detect")) {
        detection_enabled = val;
        if(!detection_enabled) {
            recognition_enabled = 0;
        }
    }
    else if(!strcmp(variable, "face_enroll")) is_enrolling = val;
    else if(!strcmp(variable, "face_recognize")) {
        recognition_enabled = val;
        if(recognition_enabled){
            detection_enabled = val;
        }
    }
    else {
        res = -1;
    }

    if(res){
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req){
    static char json_response[1024];

    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;
    *p++ = '{';

    p+=sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p+=sprintf(p, "\"quality\":%u,", s->status.quality);
    p+=sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p+=sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p+=sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p+=sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p+=sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p+=sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p+=sprintf(p, "\"awb\":%u,", s->status.awb);
    p+=sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p+=sprintf(p, "\"aec\":%u,", s->status.aec);
    p+=sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p+=sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p+=sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p+=sprintf(p, "\"agc\":%u,", s->status.agc);
    p+=sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p+=sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p+=sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p+=sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p+=sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p+=sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p+=sprintf(p, "\"vflip\":%u,", s->status.vflip);
    p+=sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p+=sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p+=sprintf(p, "\"colorbar\":%u,", s->status.colorbar);
    p+=sprintf(p, "\"face_detect\":%u,", detection_enabled);
    p+=sprintf(p, "\"face_enroll\":%u,", is_enrolling);
    p+=sprintf(p, "\"face_recognize\":%u", recognition_enabled);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
    }
    return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t status_uri = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t capture_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };

   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };


    ra_filter_init(&ra_filter, 20);
    
    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;
    
    face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
    
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}'''
st.code(code, language='c')
ur = "https://my.spline.design/3dtextbluecopy-86cab89ddbb0c385dbd3942a6b187088/"

st.button("AR",key="2")
st.write("[AR](%s)" % ur)
u = "https://my.spline.design/purpleiconsset1copy-6c433258002d4b52f5860fcdd49b4aeb/"
st.button("AR_",key="3")
st.write("[AR](%s)" % u)

l = "https://my.spline.design/keyboardrecordercopy-41ae85a220ef685a15dc6e8a59ea3596/"
st.button("AR_",key="4")
st.write("[AR](%s)" % l)    
#all_data
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression

dataimport1=pd.read_csv(r"F:\Downloads\WIFI_MLP.csv")
dataimport2=pd.read_csv(r"F:\Downloads\TV_MLP.csv")
dataimport3=pd.read_csv(r"F:\Downloads\Refrigerator_MLP.csv")
dataimport4=pd.read_csv(r"F:\Downloads\LAPTOP_SYSTEM_MLP.csv")
dataimport5=pd.read_csv(r"F:\Downloads\inverter_MLP.csv")
dataimport6=pd.read_csv(r"F:\Downloads\E-VEHICLE_MLP.csv")
dataimport7=pd.read_csv(r"F:\Downloads\ac_MLP.csv")
dataimport1['value'] = dataimport1['value'].replace(np.nan, 1)
dataimport2['value'] = dataimport2['value'].replace(np.nan, 1)
dataimport3['value'] = dataimport3['value'].replace(np.nan, 1)
dataimport4['value'] = dataimport4['value'].replace(np.nan, 1)
dataimport5['value'] = dataimport5['value'].replace(np.nan, 1)
dataimport6['value'] = dataimport6['value'].replace(np.nan, 1)
dataimport7['value'] = dataimport7['value'].replace(np.nan, 1)
Type_new = pd.Series([])
k=10001
r=10001
rk=10004
for i in range(len(dataimport1)):
    if dataimport1["value"][i] == 1:
        k
        Type_new[i]=str(k)
        k+=1
    elif dataimport1["value"][i] == 0:
        r
        Type_new[i]=str(r)
        r+=1
dataimport1.insert(4, "info_id", Type_new) 
for i in range(len(dataimport2)):
    if dataimport2["value"][i] == 1:
        k
        Type_new[i]=str(k)
        k+=1
    elif dataimport2["value"][i] == 0:
        r
        Type_new[i]=str(r)
        r+=1
dataimport2.insert(4, "info_id", Type_new) 
for i in range(len(dataimport3)):
    if dataimport3["value"][i] == 1:
        k
        Type_new[i]=str(k)
        k+=1
    elif dataimport3["value"][i] == 0:
        r
        Type_new[i]=str(r)
        r+=1
dataimport3.insert(4, "info_id", Type_new) 
for i in range(len(dataimport4)):
    if dataimport4["value"][i] == 1:
        k
        Type_new[i]=str(k)
        k+=1
    elif dataimport4["value"][i] == 0:
        r
        Type_new[i]=str(r)
        r+=1
dataimport4.insert(4, "info_id", Type_new) 
for i in range(len(dataimport5)):
    if dataimport5["value"][i] == 1:
        k
        Type_new[i]=str(k)
        k+=1
    elif dataimport5["value"][i] == 0:
        r
        Type_new[i]=str(r)
        r+=1
dataimport5.insert(4, "info_id", Type_new) 
for i in range(len(dataimport6)):
    if dataimport6["value"][i] == 1:
        k
        Type_new[i]=str(k)
        k+=1
    elif dataimport6["value"][i] == 0:
        r
        Type_new[i]=str(r)
        r+=1
dataimport6.insert(4, "info_id", Type_new) 
for i in range(len(dataimport7)):
    if dataimport7["value"][i] == 1:
        k
        Type_new[i]=str(k)
        k+=1
    elif dataimport7["value"][i] == 0:
        r
        Type_new[i]=str(r)
        r+=1
dataimport7.insert(4, "info_id", Type_new)
df1=dataimport1
df1.created_at=df1.created_at.astype(str)
df1['created_at'] = pd.to_datetime(df1.created_at.str.split(',\s*').str[0])
v=set(df1['created_at'].dt.tz_localize(tz = None))
df1["created_at"].replace=v
#st.dataframe(df1)
df2=dataimport2
df2.created_at=df2.created_at.astype(str)
df2['created_at'] = pd.to_datetime(df2.created_at.str.split(',\s*').str[0])
v=set(df2['created_at'].dt.tz_localize(tz = None))
df2["created_at"].replace=v
#st.dataframe(df2)
df3=dataimport3
df3.created_at=df3.created_at.astype(str)
df3['created_at'] = pd.to_datetime(df3.created_at.str.split(',\s*').str[0])
v=set(df3['created_at'].dt.tz_localize(tz = None))
df3["created_at"].replace=v
#st.dataframe(df3)
df4=dataimport4
df4.created_at=df4.created_at.astype(str)
df4['created_at'] = pd.to_datetime(df4.created_at.str.split(',\s*').str[0])
v=set(df4['created_at'].dt.tz_localize(tz = None))
df4["created_at"].replace=v
#st.dataframe(df4)
df5=dataimport5
df5.created_at=df5.created_at.astype(str)
df5['created_at'] = pd.to_datetime(df5.created_at.str.split(',\s*').str[0])
v=set(df5['created_at'].dt.tz_localize(tz = None))
df5["created_at"].replace=v
#st.dataframe(df5)
df6=dataimport6
df6.created_at=df6.created_at.astype(str)
df6['created_at'] = pd.to_datetime(df6.created_at.str.split(',\s*').str[0])
v=set(df6['created_at'].dt.tz_localize(tz = None))
df6["created_at"].replace=v
#st.dataframe(df6)
df7=dataimport7
df7.created_at=df7.created_at.astype(str)
df7['created_at'] = pd.to_datetime(df7.created_at.str.split(',\s*').str[0])
v=set(df7['created_at'].dt.tz_localize(tz = None))
df7["created_at"].replace=v
#st.dataframe(df7)

from sklearn.preprocessing import OrdinalEncoder
ord1 = OrdinalEncoder()
# fitting encoder
ord1.fit([df1['info_id']])
# tranforming the column after fitting
df1["info_id"]= ord1.fit_transform(df1[["info_id"]])
st.dataframe(df1)

ord2 = OrdinalEncoder()
# fitting encoder
ord2.fit([df2['info_id']])
# tranforming the column after fitting
df2["info_id"]= ord2.fit_transform(df2[["info_id"]])
st.dataframe(df2)

ord3 = OrdinalEncoder()
# fitting encoder
ord3.fit([df3['info_id']])
# tranforming the column after fitting
df3["info_id"]= ord3.fit_transform(df3[["info_id"]])
st.dataframe(df3)

ord4 = OrdinalEncoder()
# fitting encoder
ord4.fit([df4['info_id']])
# tranforming the column after fitting
df4["info_id"]= ord4.fit_transform(df4[["info_id"]])
st.dataframe(df4)

ord5 = OrdinalEncoder()
# fitting encoder
ord5.fit([df5['info_id']])
# tranforming the column after fitting
df5["info_id"]= ord5.fit_transform(df5[["info_id"]])
st.dataframe(df5)

ord6 = OrdinalEncoder()
# fitting encoder
ord6.fit([df6['info_id']])
# tranforming the column after fitting
df6["info_id"]= ord6.fit_transform(df6[["info_id"]])
st.dataframe(df6)

ord7 = OrdinalEncoder()
# fitting encoder
ord7.fit([df7['info_id']])
# tranforming the column after fitting
df7["info_id"]= ord7.fit_transform(df7[["info_id"]])
st.dataframe(df7)

df1["info_id X"] = df1["info_id"].shift()
df2["info_id X"] = df2["info_id"].shift()
df3["info_id X"] = df3["info_id"].shift()
df4["info_id X"] = df4["info_id"].shift()
df5["info_id X"] = df5["info_id"].shift()
df6["info_id X"] = df6["info_id"].shift()
df7["info_id X"] = df7["info_id"].shift()

df1["cumsum"] = (df1["info_id"] != df1["info_id X"]).cumsum()
df2["cumsum"] = (df2["info_id"] != df2["info_id X"]).cumsum()
df3["cumsum"] = (df3["info_id"] != df3["info_id X"]).cumsum()
df4["cumsum"] = (df4["info_id"] != df4["info_id X"]).cumsum()
df5["cumsum"] = (df5["info_id"] != df5["info_id X"]).cumsum()
df6["cumsum"] = (df6["info_id"] != df6["info_id X"]).cumsum()
df7["cumsum"] = (df7["info_id"] != df7["info_id X"]).cumsum()

dg1=df1.groupby((df1["info_id"] != df1["info_id"].shift()).cumsum()).agg({"created_at" : ["min", "max"]})
dg2=df2.groupby((df2["info_id"] != df2["info_id"].shift()).cumsum()).agg({"created_at" : ["min", "max"]})
dg3=df3.groupby((df3["info_id"] != df3["info_id"].shift()).cumsum()).agg({"created_at" : ["min", "max"]})
dg4=df4.groupby((df4["info_id"] != df4["info_id"].shift()).cumsum()).agg({"created_at" : ["min", "max"]})
dg5=df5.groupby((df5["info_id"] != df5["info_id"].shift()).cumsum()).agg({"created_at" : ["min", "max"]})
dg6=df6.groupby((df6["info_id"] != df6["info_id"].shift()).cumsum()).agg({"created_at" : ["min", "max"]})
dg7=df7.groupby((df7["info_id"] != df7["info_id"].shift()).cumsum()).agg({"created_at" : ["min", "max"]})

dg1.columns=['a','b']
dg2.columns=['a','b']
dg3.columns=['a','b']
dg4.columns=['a','b']
dg5.columns=['a','b']
dg6.columns=['a','b']
dg7.columns=['a','b']

difference1=dg1['b']-dg1['a']
dg1["days"]=difference1
difference2=dg2['b']-dg2['a']
dg2["days"]=difference2
difference3=dg3['b']-dg3['a']
dg3["days"]=difference3
difference4=dg4['b']-dg4['a']
dg4["days"]=difference4
difference5=dg5['b']-dg5['a']
dg5["days"]=difference5
difference6=dg6['b']-dg6['a']
dg6["days"]=difference6
difference7=dg7['b']-dg7['a']
dg7["days"]=difference7

dg1["hours"]=dg1["days"]/np.timedelta64(1,'h')
dg2["hours"]=dg2["days"]/np.timedelta64(1,'h')
dg3["hours"]=dg3["days"]/np.timedelta64(1,'h')
dg4["hours"]=dg4["days"]/np.timedelta64(1,'h')
dg5["hours"]=dg5["days"]/np.timedelta64(1,'h')
dg6["hours"]=dg6["days"]/np.timedelta64(1,'h')
dg7["hours"]=dg7["days"]/np.timedelta64(1,'h')


h1=dg1["hours"].sum()
h2=dg2["hours"].sum()
h3=dg3["hours"].sum()
h4=dg4["hours"].sum()
h5=dg5["hours"].sum()
h6=dg6["hours"].sum()
h7=dg7["hours"].sum()

dg1["units"]=dg1["hours"]*0.018
dg2["units"]=dg2["hours"]*0.110
dg3["units"]=dg3["hours"]*0.200
dg4["units"]=dg4["hours"]*0.100
dg5["units"]=dg5["hours"]*1.300
dg6["units"]=dg6["hours"]*0.960
dg7["units"]=dg7["hours"]*1.800

u1=dg1["units"].sum()
u2=dg2["units"].sum()
u3=dg3["units"].sum()
u4=dg4["units"].sum()
u5=dg5["units"].sum()
u6=dg6["units"].sum()
u7=dg7["units"].sum()

dg1["price"]=dg1["units"]*5.25
dg2["price"]=dg2["units"]*5.25
dg3["price"]=dg3["units"]*5.25
dg4["price"]=dg4["units"]*5.25
dg5["price"]=dg5["units"]*5.25
dg6["price"]=dg6["units"]*5.25
dg7["price"]=dg7["units"]*5.25

p1=dg1["price"].sum()
p2=dg2["price"].sum()
p3=dg3["price"].sum()
p4=dg4["price"].sum()
p5=dg5["price"].sum()
p6=dg6["price"].sum()
p7=dg7["price"].sum()

data__={"TOTAL_HOURS":[h1,h2,h3,h4,h5,h6,h7],"TOTAL_UNITS":[u1,u2,u3,u4,u5,u6,u7],"TOTAL_PRICE":[p1,p2,p3,p4,p5,p6,p7]}
datalop=pd.DataFrame(data__)
#st.dataframe(datalop)

dataset=datalop[["TOTAL_HOURS","TOTAL_UNITS"]]
st.dataframe(dataset)
#st.area_chart(data=dataset,x=datalop["TOTAL_HOURS"], y=datalop["TOTAL_UNITS"], width=0, height=0, use_container_width=True)


dataset.plot(x='TOTAL_UNITS', y='TOTAL_HOURS', style='*')
plt.title('hours vs price')
plt.xlabel('hours')
plt.ylabel('price')
#plt.show()
X = dataset.iloc[:, :-1].values
y = dataset.iloc[:, 1].values





X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=1/3, random_state=0)

regressor = LinearRegression()
regressor.fit(X_train, y_train)
print(regressor.intercept_)
print(regressor.coef_)
y_pred = regressor.predict(X_test)
x_pred = regressor.predict(X_train)
dk = pd.DataFrame({'Actual': y_test, 'Predicted': y_pred})
print(dk)
corr = dk.corr()
print(corr)
from sklearn import metrics
print('Mean Absolute Error:', metrics.mean_absolute_error(y_test, y_pred))
print('Mean Squared Error:', metrics.mean_squared_error(y_test, y_pred))
print('Root Mean Squared Error:', np.sqrt(metrics.mean_squared_error(y_test, y_pred)))


HOURS = st.slider('HOURS ?', 0, 3000, 25)
st.write("HOURS", HOURS)        

POWER = st.slider('POWER ?', 0, 3000, 25)
st.write("POWER", POWER)     
fh=np.array([POWER,HOURS])
fh=np.array([POWER,HOURS]).reshape(-1,1)
W = fh.astype(float)
K=regressor.predict(W)
print(K)
st.write(K) 
        
