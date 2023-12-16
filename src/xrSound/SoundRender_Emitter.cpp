#include "stdafx.h"

#include "SoundRender_Emitter.h"
#include "SoundRender_Core.h"
#include "SoundRender_Scene.h"
#include "SoundRender_Source.h"
#include "SoundRender_TargetA.h"

extern u32 psSoundModel;
extern float psSoundVEffects;

void CSoundRender_Emitter::set_position(const Fvector& pos)
{
    if (source()->channels_num() == 1)
        p_source.update_position(pos);
    else
        p_source.update_position({});

#ifdef USE_PHONON
    const IPLCoordinateSpace3 sourceCoordinates
    {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
        reinterpret_cast<const IPLVector3&>(pos)
    };

    IPLSimulationInputs inputs{};
    inputs.flags = IPL_SIMULATIONFLAGS_DIRECT;
    inputs.directFlags = static_cast<IPLDirectSimulationFlags>(
        IPL_DIRECTSIMULATIONFLAGS_DISTANCEATTENUATION |
        IPL_DIRECTSIMULATIONFLAGS_AIRABSORPTION |
        IPL_DIRECTSIMULATIONFLAGS_DIRECTIVITY |
        IPL_DIRECTSIMULATIONFLAGS_OCCLUSION |
        IPL_DIRECTSIMULATIONFLAGS_TRANSMISSION
    );
    inputs.source = sourceCoordinates;
    inputs.occlusionType = IPL_OCCLUSIONTYPE_RAYCAST;

    iplSourceSetInputs(ipl_source, IPL_SIMULATIONFLAGS_DIRECT, &inputs);
#endif

    bMoved = true;
}

void CSoundRender_Emitter::set_frequency(float scale)
{
    VERIFY(_valid(scale));
    if (_valid(scale))
        p_source.freq = scale;
}

// Перемотка звука на заданную секунду [rewind snd to target time] --#SM+#--
void CSoundRender_Emitter::set_time(float t)
{
    VERIFY2(get_length_sec() >= t, "set_time: time is bigger than length of sound");
    clamp(t, 0.0f, get_length_sec());
    fTimeToRewind = t;
}

CSoundRender_Emitter::CSoundRender_Emitter(CSoundRender_Scene* s)
    : scene(s)
{
#ifdef DEBUG
    static u32 incrementalID = 0;
    dbg_ID = ++incrementalID;
#endif
    target = nullptr;
    //source = nullptr;
    owner_data = nullptr;
    smooth_volume = 1.f;
    occluder_volume = 1.f;
    fade_volume = 1.f;
    occluder[0].set(0, 0, 0);
    occluder[1].set(0, 0, 0);
    occluder[2].set(0, 0, 0);
    m_current_state = stStopped;
    set_cursor(0);
    bMoved = true;
    b2D = false;
    bStopping = false;
    bRewind = false;
    bIgnoringTimeFactor = false;
    iPaused = 0;
    fTimeStarted = 0.0f;
    fTimeToStop = 0.0f;
    fTimeToPropagade = 0.0f;
    fTimeToRewind = 0.0f; //--#SM+#--
    marker = 0xabababab;
    starting_delay = 0.f;
    priority_scale = 1.f;
    m_cur_handle_cursor = 0;

#ifdef USE_PHONON
    IPLSourceSettings sourceSettings{ IPL_SIMULATIONFLAGS_DIRECT }; // this enables occlusion/transmission simulator for this source
    iplSourceCreate(scene->ipl_simulator, &sourceSettings, &ipl_source);
    iplSourceAdd(ipl_source, scene->ipl_simulator);

    iplSimulatorCommit(scene->ipl_simulator);
#endif
}

CSoundRender_Emitter::~CSoundRender_Emitter()
{
#ifdef USE_PHONON
    iplSourceRemove(ipl_source, scene->ipl_simulator);
    iplSourceRelease(&ipl_source);

    iplSimulatorCommit(scene->ipl_simulator);
#endif

    // try to release dependencies, events, for example
    Event_ReleaseOwner();
}

//////////////////////////////////////////////////////////////////////
void CSoundRender_Emitter::Event_ReleaseOwner()
{
    if (!owner_data)
        return;

    auto& events = scene->get_events();

    for (u32 it = 0; it < events.size(); it++)
    {
        if (owner_data == events[it].first)
        {
            events.erase(events.begin() + it);
            it--;
        }
    }
}

void CSoundRender_Emitter::Event_Propagade()
{
    fTimeToPropagade += ::Random.randF(s_f_def_event_pulse - 0.030f, s_f_def_event_pulse + 0.030f);
    if (!owner_data)
        return;
    if (!owner_data->g_type)
        return;
    if (!owner_data->g_object)
        return;
    if (!scene->get_events_handler())
        return;

    VERIFY(_valid(p_source.volume));
    // Calculate range
    const float clip = p_source.max_ai_distance * p_source.volume;
    const float range = std::min(p_source.max_ai_distance, clip);
    if (range < 0.1f)
        return;

    // Inform objects
    scene->get_events().emplace_back(owner_data, range);
}

void CSoundRender_Emitter::switch_to_2D()
{
    b2D = true;
    set_priority(100.f);
}

void CSoundRender_Emitter::switch_to_3D()
{
    b2D = false;
}

u32 CSoundRender_Emitter::play_time()
{
    if (m_current_state == stPlaying || m_current_state == stPlayingLooped || m_current_state == stSimulating ||
        m_current_state == stSimulatingLooped)
        return iFloor((SoundRender->fTimer_Value - fTimeStarted) * 1000.0f);
    return 0;
}

#include "SoundRender_Source.h" // XXX: remove maybe
void CSoundRender_Emitter::set_cursor(u32 p)
{
    m_stream_cursor = p;

    if (owner_data._get() && owner_data->fn_attached[0].size())
    {
        u32 bt = ((CSoundRender_Source*)owner_data->handle)->dwBytesTotal;
        if (m_stream_cursor >= m_cur_handle_cursor + bt)
        {
            SoundRender->i_destroy_source((CSoundRender_Source*)owner_data->handle);
            owner_data->handle = SoundRender->i_create_source(owner_data->fn_attached[0].c_str());
            owner_data->fn_attached[0] = owner_data->fn_attached[1];
            owner_data->fn_attached[1] = "";
            m_cur_handle_cursor = get_cursor(true);

            if (target)
                ((CSoundRender_TargetA*)target)->source_changed();
        }
    }
}

u32 CSoundRender_Emitter::get_cursor(bool b_absolute) const
{
    if (b_absolute)
        return m_stream_cursor;
    VERIFY(m_stream_cursor - m_cur_handle_cursor >= 0);
    return m_stream_cursor - m_cur_handle_cursor;
}

void CSoundRender_Emitter::move_cursor(int offset) { set_cursor(get_cursor(true) + offset); }
