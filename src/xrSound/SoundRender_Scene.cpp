#include "stdafx.h"

#include "Common/LevelStructure.hpp"
#include "xrCore/Threading/TaskManager.hpp"
#include "xrCDB/Intersect.hpp"
#include "xrMaterialSystem/GameMtlLib.h"

#include "SoundRender_Core.h"
#include "SoundRender_Scene.h"
#include "SoundRender_Emitter.h"

CSoundRender_Scene::CSoundRender_Scene()
{
#ifdef USE_PHONON
    IPLSceneSettings sceneSettings
    {
        IPL_SCENETYPE_DEFAULT,
        nullptr, nullptr, nullptr, nullptr,
        this,
        nullptr, nullptr
    };
    iplSceneCreate(SoundRender->ipl_context, &sceneSettings, &ipl_scene);

    IPLSimulationSettings simulationSettings
    {
        static_cast<IPLSimulationFlags>(IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS),
        IPL_SCENETYPE_DEFAULT,
        IPL_REFLECTIONEFFECTTYPE_CONVOLUTION,
        64,
        128,
        32,
        2.0f,
        1,
        8,
        2,
        5,
        32,
        44100, 2205,
        nullptr, nullptr, nullptr,
    };
    iplSimulatorCreate(SoundRender->ipl_context, &simulationSettings, &ipl_simulator);

    iplSimulatorSetScene(ipl_simulator, ipl_scene);
    iplSimulatorCommit(ipl_simulator);
#endif
}

CSoundRender_Scene::~CSoundRender_Scene()
{
    stop_emitters();

    set_geometry_som(nullptr);

    // remove emitters
    for (auto& emit : s_emitters)
        xr_delete(emit);
    s_emitters.clear();

    set_geometry_occ(nullptr, {});

#ifdef USE_PHONON
    iplProbeBatchRelease(&ipl_scene_probes);
    iplSimulatorRelease(&ipl_simulator);
    iplSceneRelease(&ipl_scene);
#endif
}

void CSoundRender_Scene::stop_emitters() const
{
    for (const auto& emit : s_emitters)
        emit->stop(false);
}

int CSoundRender_Scene::pause_emitters(bool pauseState)
{
    m_iPauseCounter += pauseState ? +1 : -1;
    VERIFY(m_iPauseCounter >= 0);

    for (const auto& emit : s_emitters)
        emit->pause(pauseState, pauseState ? m_iPauseCounter : m_iPauseCounter + 1);

    return m_iPauseCounter;
}

void CSoundRender_Scene::set_handler(sound_event* E) { sound_event_handler = E; }

void CSoundRender_Scene::set_geometry_occ(CDB::MODEL* M, const Fbox& aabb)
{
#ifdef USE_PHONON
    if (geom_MODEL != M && ipl_scene_mesh)
    {
        iplStaticMeshRemove(ipl_scene_mesh, ipl_scene);
        iplStaticMeshRelease(&ipl_scene_mesh);
    }
    if (M)
    {
        const auto tris = M->get_tris();
        const auto tris_count = M->get_tris_count();

        const auto verts = M->get_verts();
        const auto verts_count = M->get_verts_count();

        auto* temp_tris = xr_alloc<IPLTriangle>(tris_count);
        auto* temp_mat_idx = xr_alloc<IPLint32>(tris_count);

        IPLMaterial materials[] =
        {
            { { 0.1f, 0.1f, 0.1f }, 0.5f, { 0.2f, 0.2f, 0.2f } },
            { { 0.1f, 0.2f, 0.3f }, 0.05f, { 0.1f, 0.05f, 0.030f } },
        };

        for (size_t i = 0; i < tris_count; ++i)
        {
            temp_tris[i] = reinterpret_cast<IPLTriangle&>(tris[i].verts);
            temp_mat_idx[i] = 0;
        }

        IPLStaticMeshSettings staticMeshSettings
        {
            verts_count, tris_count, std::size(materials),
            reinterpret_cast<IPLVector3*>(verts), temp_tris,
            temp_mat_idx, materials
        };

        iplStaticMeshCreate(ipl_scene, &staticMeshSettings, &ipl_scene_mesh);
        xr_free(temp_mat_idx);
        xr_free(temp_tris);

        iplStaticMeshAdd(ipl_scene_mesh, ipl_scene);
        iplSceneCommit(ipl_scene);

        const auto transform = aabb.get_xform();

        IPLProbeGenerationParams probeParams
        {
            IPL_PROBEGENERATIONTYPE_UNIFORMFLOOR,
            2.0f, 1.5f,
            reinterpret_cast<const IPLMatrix4x4&>(transform)
        };

        IPLProbeArray probeArray{};
        iplProbeArrayCreate(SoundRender->ipl_context, &probeArray);
        iplProbeArrayGenerateProbes(probeArray, ipl_scene, &probeParams);

        constexpr IPLBakedDataIdentifier identifier
        {
            IPL_BAKEDDATATYPE_REFLECTIONS,
            IPL_BAKEDDATAVARIATION_REVERB,
            {},
        };

        iplProbeBatchCreate(SoundRender->ipl_context, &ipl_scene_probes);
        iplProbeBatchAddProbeArray(ipl_scene_probes, probeArray);
        iplProbeBatchCommit(ipl_scene_probes);

        IPLPathBakeParams pathingBakeParams
        {
            ipl_scene, ipl_scene_probes, identifier,
            2, 2.0f, 0.5f,
            25.0f, 50.0f,
            s32(TaskScheduler->GetWorkersCount())
        };

        //iplPathBakerBake(SoundRender->ipl_context, &pathingBakeParams, [](IPLfloat32 progress, void*)
        //{
        //    Msg("SOUND: SteamAudio: path baker progress: %f", progress);
        //}, nullptr);

        IPLReflectionsBakeParams reflectionsBakeParams
        {
            ipl_scene, ipl_scene_probes, IPL_SCENETYPE_DEFAULT, identifier,
            IPL_REFLECTIONSBAKEFLAGS_BAKECONVOLUTION,
            128, 128, 32,
            2.0f, 2.0f,
            1, s32(TaskScheduler->GetWorkersCount()), 0,
            1.0f,
            0, nullptr, nullptr
        };

        iplReflectionsBakerBake(SoundRender->ipl_context, &reflectionsBakeParams, [](IPLfloat32 progress, void*)
        {
            Msg("SOUND: SteamAudio: path baker progress: %f", progress);
        }, nullptr);

        iplSimulatorAddProbeBatch(ipl_simulator, ipl_scene_probes);
        iplSimulatorCommit(ipl_simulator);

        iplProbeArrayRelease(&probeArray);
    }
#endif

    geom_MODEL = M;
}

void CSoundRender_Scene::set_geometry_som(IReader* I)
{
    xr_delete(geom_SOM);
    if (nullptr == I)
        return;

    // check version
    R_ASSERT(I->find_chunk(0));
    [[maybe_unused]] const u32 version = I->r_u32();
    VERIFY2(version == 0, "Invalid SOM version");

    struct SOM_poly
    {
        Fvector3 v1;
        Fvector3 v2;
        Fvector3 v3;
        u32 b2sided;
        float occ;
    };

    CDB::Collector CL;
    {
        // load geometry
        IReader* geom = I->open_chunk(1);
        VERIFY2(geom, "Corrupted SOM file");
        if (!geom)
            return;

        // Load tris and merge them
        const auto begin = static_cast<SOM_poly*>(geom->pointer());
        const auto end = static_cast<SOM_poly*>(geom->end());
        for (SOM_poly* poly = begin; poly != end; ++poly)
        {
            CL.add_face_packed_D(poly->v1, poly->v2, poly->v3, *(u32*)&poly->occ, 0.01f);
            if (poly->b2sided)
                CL.add_face_packed_D(poly->v3, poly->v2, poly->v1, *(u32*)&poly->occ, 0.01f);
        }
        geom->close();
    }

    // Create AABB-tree
    geom_SOM = xr_new<CDB::MODEL>();
    geom_SOM->build(CL.getV(), int(CL.getVS()), CL.getT(), int(CL.getTS()));
}

void CSoundRender_Scene::play(ref_sound& S, IGameObject* O, u32 flags, float delay)
{
    if (!SoundRender->bPresent || !S._handle())
        return;
    S->g_object = O;
    if (S._feedback())
        ((CSoundRender_Emitter*)S._feedback())->rewind();
    else
        i_play(S, flags, delay);

    if (flags & sm_2D || S._handle()->channels_num() == 2)
        S._feedback()->switch_to_2D();

    S._feedback()->set_ignore_time_factor(flags & sm_IgnoreTimeFactor);
}

void CSoundRender_Scene::play_no_feedback(
    ref_sound& S, IGameObject* O, u32 flags, float delay, Fvector* pos, float* vol, float* freq, Fvector2* range)
{
    if (!SoundRender->bPresent || !S._handle())
        return;
    const ref_sound orig = S;
    S._set(xr_new<CSound>());
    S->handle = orig->handle;
    S->g_type = orig->g_type;
    S->g_object = O;
    S->dwBytesTotal = orig->dwBytesTotal;
    S->fTimeTotal = orig->fTimeTotal;
    S->fn_attached[0] = orig->fn_attached[0];
    S->fn_attached[1] = orig->fn_attached[1];

    i_play(S, flags, delay);

    if (flags & sm_2D || S._handle()->channels_num() == 2)
        S._feedback()->switch_to_2D();

    if (pos)
        S._feedback()->set_position(*pos);
    if (freq)
        S._feedback()->set_frequency(*freq);
    if (range)
        S._feedback()->set_range((*range)[0], (*range)[1]);
    if (vol)
        S._feedback()->set_volume(*vol);
    S = orig;
}

void CSoundRender_Scene::play_at_pos(ref_sound& S, IGameObject* O, const Fvector& pos, u32 flags, float delay)
{
    if (!SoundRender->bPresent || !S._handle())
        return;
    S->g_object = O;
    if (S._feedback())
        ((CSoundRender_Emitter*)S._feedback())->rewind();
    else
        i_play(S, flags, delay);

    S._feedback()->set_position(pos);

    if (flags & sm_2D || S._handle()->channels_num() == 2)
        S._feedback()->switch_to_2D();

    S._feedback()->set_ignore_time_factor(flags & sm_IgnoreTimeFactor);
}

CSoundRender_Emitter* CSoundRender_Scene::i_play(ref_sound& S, u32 flags, float delay)
{
    VERIFY(!S->feedback);
    CSoundRender_Emitter* E = s_emitters.emplace_back(xr_new<CSoundRender_Emitter>(this));
    S->feedback = E;
    E->start(S, flags, delay);
    return E;
}

void CSoundRender_Scene::update()
{
    s_events_prev_count = s_events.size();

    for (auto& [sound, range] : s_events)
        sound_event_handler(sound, range);

    s_events.clear();

#ifdef USE_PHONON
    iplSimulatorRunDirect(ipl_simulator);
    if (ipl_scene_mesh)
    {
        iplSimulatorRunReflections(ipl_simulator);
        //iplSimulatorRunPathing(ipl_simulator);
    }
#endif
}

void CSoundRender_Scene::object_relcase(IGameObject* obj)
{
    if (obj)
    {
        for (const auto& emit : s_emitters)
        {
            if (emit)
                if (emit->owner_data)
                    if (obj == emit->owner_data->g_object)
                        emit->owner_data->g_object = 0;
        }
    }
}

float CSoundRender_Scene::get_occlusion_to(const Fvector& hear_pt, const Fvector& snd_pt, float dispersion)
{
    float occ_value = 1.f;

    if (nullptr != geom_SOM)
    {
        // Calculate RAY params
        Fvector pos, dir;
        pos.random_dir();
        pos.mul(dispersion);
        pos.add(snd_pt);
        dir.sub(pos, hear_pt);
        const float range = dir.magnitude();
        dir.div(range);

        geom_DB.ray_query(CDB::OPT_CULL, geom_SOM, hear_pt, dir, range);
        const auto r_cnt = geom_DB.r_count();
        CDB::RESULT* _B = geom_DB.r_begin();
        if (0 != r_cnt)
        {
            for (size_t k = 0; k < r_cnt; k++)
            {
                CDB::RESULT* R = _B + k;
                occ_value *= *reinterpret_cast<float*>(&R->dummy);
            }
        }
    }
    return occ_value;
}

float CSoundRender_Scene::get_occlusion(const Fvector& P, float R, Fvector* occ)
{
    float occ_value = 1.f;

    // Calculate RAY params
    const Fvector base = SoundRender->listener_position();
    Fvector pos, dir;
    pos.random_dir();
    pos.mul(R);
    pos.add(P);
    dir.sub(pos, base);
    const float range = dir.magnitude();
    dir.div(range);

    if (nullptr != geom_MODEL)
    {
        bool bNeedFullTest = true;
        // 1. Check cached polygon
        float _u, _v, _range;
        if (CDB::TestRayTri(base, dir, occ, _u, _v, _range, true))
            if (_range > 0 && _range < range)
            {
                occ_value = psSoundOcclusionScale;
                bNeedFullTest = false;
            }
        // 2. Polygon doesn't picked up - real database query
        if (bNeedFullTest)
        {
            geom_DB.ray_query(CDB::OPT_ONLYNEAREST, geom_MODEL, base, dir, range);
            if (0 != geom_DB.r_count())
            {
                // cache polygon
                const CDB::RESULT* R2 = geom_DB.r_begin();
                const CDB::TRI& T = geom_MODEL->get_tris()[R2->id];
                const Fvector* V = geom_MODEL->get_verts();
                occ[0].set(V[T.verts[0]]);
                occ[1].set(V[T.verts[1]]);
                occ[2].set(V[T.verts[2]]);

                const SGameMtl* mtl = GMLib.GetMaterialByIdx(T.material);
                const float occlusion = fis_zero(mtl->fSndOcclusionFactor) ? 0.1f : mtl->fSndOcclusionFactor;
                occ_value = psSoundOcclusionScale * occlusion;
            }
        }
    }
    if (nullptr != geom_SOM)
    {
        geom_DB.ray_query(CDB::OPT_CULL, geom_SOM, base, dir, range);
        const auto r_cnt = geom_DB.r_count();
        CDB::RESULT* _B = geom_DB.r_begin();
        if (0 != r_cnt)
        {
            for (size_t k = 0; k < r_cnt; k++)
            {
                CDB::RESULT* R2 = _B + k;
                occ_value *= *reinterpret_cast<float*>(&R2->dummy);
            }
        }
    }
    return occ_value;
}
