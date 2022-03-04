#pragma once
#include "vec.h"
#include "Material.h"
#include "Object.h"
#include "Scene.h"

enum class VertexType { Camera, Light, Surface };

class Vertex {
public:
	vec2 pos;
	double pdfFwd = 0, pdfRev = 0;
	Object* obj;
	vec2 normal;
	VertexType type;
    vec2 p() const { return pos; }
    vec2 n() const { return normal; }

    double ConvertDensity(double pdf, const Vertex& next) const {
        vec2 w = next.p() - p();
        if (w.length() <= 1e-10) 
            return 0.0;
        double invDist = 1.0 / w.length();
        if (next.type != VertexType::Camera) {
            pdf *= abs(dot(next.n(), w.normalized()));
        }
        return pdf * invDist;
    }

    double Pdf(const Scene& scene, const Vertex* prev,
        const Vertex& next) const {
        if (type == VertexType::Light)
            return PdfLight(scene, next);
        if (type == VertexType::Camera) {
            return ConvertDensity(1.0, next);
        }
        vec2 wp, wn = (next.p() - p()).normalized();
        if (prev)
            wp = (prev->p() - p()).normalized();
        double pdf;
        pdf = obj->mat->pdf(wp, wn);
        //if (type == VertexType::Camera)
        //    ei.camera->Pdf_We(ei.SpawnRay(wn), &unused, &pdf);
        //else if (type == VertexType::Surface)
        //    pdf = si.bsdf->Pdf(wp, wn);
        //else if (type == VertexType::Medium)
        //    pdf = mi.phase->p(wp, wn);


        return ConvertDensity(pdf, next);

    }
    
    double PdfLightOrigin(const Scene& scene, const Vertex& v) const {
        //// << Return solid angle density for non - infinite light sources >>
        //double pdfPos, pdfDir, pdfChoice = 1;
        ////<< Get pointer light to the light source at the vertex >>
        //const AreaLight * light = (AreaLight*)obj;
        //light.mat->Pdf_Le(Ray(p(), w, time()), ng(), &pdfPos, &pdfDir);
        return 1.0 / (0.4);

    }

    double PdfLight(const Scene& scene, const Vertex& v) const {
        vec2 w = v.p() - p();
        double invDist = 1 / w.length();
        w = w.normalized();
        double pdf;
        // << Get pointer light to the light source at the vertex >>
        const LightMaterial* light = (LightMaterial*)(obj->mat);


        // << Compute sampling density for non - infinite light sources >>
        vec2 n = vec2(1.0, 0.0);
        vec2 tan = vec2(-n[1], n[0]);
        vec2 wLocal = vec2(dot(w, tan), dot(w, n));
        double pdfDir = light->Pdf_Le(wLocal.normalized());
        pdf = pdfDir * invDist;

        pdf *= abs(dot(v.n(), w));
        return pdf;
    }

    vec3 f(const Vertex& prev, const Vertex& next) const {
        vec2 wi = (next.p() - p());
        vec2 wo = (prev.p() - p()).normalized();
        if (wi.length() < 1e-10)
        {
            return 0.0;
        }
        wi = wi.normalized();
        if (obj == nullptr) { return 0.0; }
        return obj->mat->bsdf(wo, wi);
    }

    vec3 Le(const Vertex& next) const {
        vec2 w = (next.p() - p()).normalized(); 
        return ((LightMaterial*)(obj->mat))->Le(w);
    }

    vec3 We(const Vertex& next) const {
        return 1.0;
    }
};
