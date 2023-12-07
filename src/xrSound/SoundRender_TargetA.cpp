#include "stdafx.h"

#include "SoundRender_TargetA.h"
#include "SoundRender_Emitter.h"
#include "SoundRender_Source.h"

xr_vector<u8> g_target_temp_data;
static IPLAudioSettings audioSettings{ 44100, 1024 };

CSoundRender_TargetA::CSoundRender_TargetA() : CSoundRender_Target()
{
    cache_gain = 0.f;
    cache_pitch = 1.f;
    pSource = 0;
    buf_block = 0;
}

CSoundRender_TargetA::~CSoundRender_TargetA() {}

bool CSoundRender_TargetA::_initialize()
{
    inherited::_initialize();
    // initialize buffer
    A_CHK(alGenBuffers(sdef_target_count, pBuffers));
    alGenSources(1, &pSource);
    ALenum error = alGetError();
    if (AL_NO_ERROR == error)
    {
        A_CHK(alSourcei(pSource, AL_LOOPING, AL_FALSE));
        A_CHK(alSourcef(pSource, AL_MIN_GAIN, 0.f));
        A_CHK(alSourcef(pSource, AL_MAX_GAIN, 1.f));
        A_CHK(alSourcef(pSource, AL_GAIN, cache_gain));
        A_CHK(alSourcef(pSource, AL_PITCH, cache_pitch));
        return true;
    }
    Msg("! sound: OpenAL: Can't create source. Error: %s.", static_cast<pcstr>(alGetString(error)));
    return false;
}

void CSoundRender_TargetA::_destroy()
{
    // clean up target
    if (alIsSource(pSource))
        alDeleteSources(1, &pSource);
    A_CHK(alDeleteBuffers(sdef_target_count, pBuffers));
}

void CSoundRender_TargetA::_restart()
{
    _destroy();
    _initialize();
}

void CSoundRender_TargetA::start(CSoundRender_Emitter* E)
{
    inherited::start(E);

    // Calc storage
    const auto wvf = E->source()->m_wformat;
    buf_block = sdef_target_block * wvf.nAvgBytesPerSec / 1000;
    g_target_temp_data.resize(buf_block);

#ifdef USE_PHONON
    iplAudioBufferFree(SoundRender->ipl_context, &ipl_buffer_deinterleaved);
    iplAudioBufferFree(SoundRender->ipl_context, &ipl_buffer_interleaved);
    iplDirectEffectRelease(&ipl_effect);

    const IPLint32 samples_per_buf_block = buf_block / (wvf.wBitsPerSample * wvf.nChannels);
    audioSettings.frameSize = samples_per_buf_block;

    iplAudioBufferAllocate(SoundRender->ipl_context, wvf.nChannels, samples_per_buf_block,
        &ipl_buffer_deinterleaved);
    iplAudioBufferAllocate(SoundRender->ipl_context, wvf.nChannels, samples_per_buf_block,
        &ipl_buffer_interleaved);

    IPLDirectEffectSettings effectSettings{ wvf.nChannels };
    iplDirectEffectCreate(SoundRender->ipl_context, &audioSettings, &effectSettings, &ipl_effect);
#endif
}

void CSoundRender_TargetA::render()
{
    for (ALuint pBuffer : pBuffers)
        fill_block(pBuffer);

    A_CHK(alSourceQueueBuffers(pSource, sdef_target_count, pBuffers));
    A_CHK(alSourcePlay(pSource));

    inherited::render();
}

void CSoundRender_TargetA::stop()
{
    if (rendering)
    {
        A_CHK(alSourceStop(pSource));
        A_CHK(alSourcei(pSource, AL_BUFFER, 0));
        A_CHK(alSourcei(pSource, AL_SOURCE_RELATIVE, TRUE));
    }
    inherited::stop();
}

void CSoundRender_TargetA::rewind()
{
    inherited::rewind();

    A_CHK(alSourceStop(pSource));
    A_CHK(alSourcei(pSource, AL_BUFFER, 0));
    for (ALuint pBuffer : pBuffers)
        fill_block(pBuffer);
    A_CHK(alSourceQueueBuffers(pSource, sdef_target_count, pBuffers));
    A_CHK(alSourcePlay(pSource));
}

void CSoundRender_TargetA::update()
{
    inherited::update();

    ALint processed, state;
    ALenum error;

    /* Get relevant source info */
    alGetSourcei(pSource, AL_SOURCE_STATE, &state);
    alGetSourcei(pSource, AL_BUFFERS_PROCESSED, &processed);
    if ((error = alGetError()) != AL_NO_ERROR)
    {
        Msg("! %s:: source state check failed (0x%d)", __FUNCTION__, error);
        return;
    }

    while (processed > 0)
    {
        ALuint BufferID;
        A_CHK(alSourceUnqueueBuffers(pSource, 1, &BufferID));
        fill_block(BufferID);
        A_CHK(alSourceQueueBuffers(pSource, 1, &BufferID));
        processed--;
        if ((error = alGetError()) != AL_NO_ERROR)
        {
            Msg("! %s:: buffering data failed (0x%d)", __FUNCTION__, error);
            return;
        }
    }

    /* Make sure the source hasn't underrun */
    if (state != AL_PLAYING && state != AL_PAUSED)
    {
        ALint queued;

        /* If no buffers are queued, playback is finished */
        alGetSourcei(pSource, AL_BUFFERS_QUEUED, &queued);
        if (queued == 0)
            return;

        alSourcePlay(pSource);
        if ((error = alGetError()) != AL_NO_ERROR)
        {
            Msg("! %s:: restarting playback failed (0x%d)", __FUNCTION__, error);
            return;
        }
    }
}

void CSoundRender_TargetA::fill_parameters()
{
    [[maybe_unused]] CSoundRender_Emitter* SE = m_pEmitter;
    VERIFY(SE);

    inherited::fill_parameters();

    // 3D params
    VERIFY2(m_pEmitter, SE->source()->file_name());
    A_CHK(alSourcef(pSource, AL_REFERENCE_DISTANCE, m_pEmitter->p_source.min_distance));

    VERIFY2(m_pEmitter, SE->source()->file_name());
    A_CHK(alSourcef(pSource, AL_MAX_DISTANCE, m_pEmitter->p_source.max_distance));

    VERIFY2(m_pEmitter, SE->source()->file_name());
    A_CHK(alSource3f(pSource, AL_POSITION, m_pEmitter->p_source.position.x, m_pEmitter->p_source.position.y,
        -m_pEmitter->p_source.position.z));

    VERIFY2(m_pEmitter, SE->source()->file_name());
    A_CHK(alSource3f(pSource, AL_VELOCITY, m_pEmitter->p_source.velocity.x, m_pEmitter->p_source.velocity.y,
        -m_pEmitter->p_source.velocity.z));

    VERIFY2(m_pEmitter, SE->source()->file_name());
    A_CHK(alSourcei(pSource, AL_SOURCE_RELATIVE, m_pEmitter->b2D));

    A_CHK(alSourcef(pSource, AL_ROLLOFF_FACTOR, psSoundRolloff));

    VERIFY2(m_pEmitter, SE->source()->file_name());
    float _gain = m_pEmitter->smooth_volume;
    clamp(_gain, EPS_S, 1.f);
    if (!fsimilar(_gain, cache_gain, 0.01f))
    {
        cache_gain = _gain;
        A_CHK(alSourcef(pSource, AL_GAIN, _gain));
    }

    VERIFY2(m_pEmitter, SE->source()->file_name());

    float _pitch = m_pEmitter->p_source.freq;
    if (!m_pEmitter->bIgnoringTimeFactor)
        _pitch *= psSoundTimeFactor; //--#SM+#-- Correct sound "speed" by time factor
    clamp(_pitch, EPS_L, 100.f); //--#SM+#-- Increase sound frequency (speed) limit

    if (!fsimilar(_pitch, cache_pitch))
    {
        cache_pitch = _pitch;
        A_CHK(alSourcef(pSource, AL_PITCH, _pitch));
    }
    VERIFY2(m_pEmitter, SE->source()->file_name());
}

void CSoundRender_TargetA::fill_block(ALuint BufferID)
{
    R_ASSERT(m_pEmitter);

    m_pEmitter->fill_block(&g_target_temp_data.front(), buf_block);

    const auto& wvf = m_pEmitter->source()->m_wformat;
    const bool mono = wvf.nChannels == 1;

#ifdef USE_PHONON
    IPLSimulationOutputs outputs{};
    iplSourceGetOutputs(m_pEmitter->ipl_source, IPL_SIMULATIONFLAGS_DIRECT, &outputs);

    iplAudioBufferDeinterleave(SoundRender->ipl_context,
        (IPLfloat32*)g_target_temp_data.data(),
        &ipl_buffer_deinterleaved);

    iplDirectEffectApply(ipl_effect, &outputs.direct, &ipl_buffer_deinterleaved, &ipl_buffer_interleaved);

    iplAudioBufferInterleave(SoundRender->ipl_context, &ipl_buffer_interleaved, (IPLfloat32*)g_target_temp_data.data());
#endif

    ALuint format;
#if AL_EXT_float32
    if (wvf.wFormatTag == WAVE_FORMAT_IEEE_FLOAT)
        format = mono ? AL_FORMAT_MONO_FLOAT32 : AL_FORMAT_STEREO_FLOAT32;
    else
#endif
    {
        format = mono ? AL_FORMAT_MONO16 : AL_FORMAT_STEREO16;
    }

    A_CHK(alBufferData(
        BufferID, format, &g_target_temp_data.front(), buf_block, m_pEmitter->source()->m_wformat.nSamplesPerSec));
}
void CSoundRender_TargetA::source_changed()
{
    detach();
    attach();
}
