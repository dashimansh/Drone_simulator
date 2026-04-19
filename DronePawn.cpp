#include "DronePawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Kismet/GameplayStatics.h"

ADronePawn::ADronePawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // ── Body ────────────────────────────────────────────────
    DroneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneMesh"));
    RootComponent = DroneMesh;
    DroneMesh->SetSimulatePhysics(false);
    DroneMesh->SetRelativeRotation(FRotator(0, -90, 0));

    // ── Rotors ──────────────────────────────────────────────
    auto MakeRotor = [&](const FName& Name, FVector Offset) -> UStaticMeshComponent*
    {
        auto* R = CreateDefaultSubobject<UStaticMeshComponent>(Name);
        R->SetupAttachment(DroneMesh);
        R->SetRelativeLocation(Offset);
        R->SetRelativeRotation(FRotator(0, 0, 90));
        return R;
    };

    Rotor_FL = MakeRotor(TEXT("Rotor_FL"), FVector(8, 24, 26));
    Rotor_FR = MakeRotor(TEXT("Rotor_FR"), FVector(8, -24, 26));
    Rotor_BL = MakeRotor(TEXT("Rotor_BL"), FVector(-10, 7, 24));
    Rotor_BR = MakeRotor(TEXT("Rotor_BR"), FVector(-10, -7, 24));

    // ── Third Person Camera ─────────────────────────────────
    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(DroneMesh);
    SpringArm->TargetArmLength = 200.f;
    SpringArm->SetRelativeLocation(FVector(0, 0, 0));
    SpringArm->SetRelativeRotation(FRotator(-20, 0, 0));
    SpringArm->bUsePawnControlRotation = false;
    SpringArm->bInheritYaw = true;
    SpringArm->bInheritPitch = false;
    SpringArm->bInheritRoll = false;

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);
    Camera->SetActive(true);

    // ── FPV Camera ──────────────────────────────────────────
    FPVCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FPVCamera"));
    FPVCamera->SetupAttachment(DroneMesh);
    FPVCamera->SetRelativeLocation(FVector(15, 0, 5));
    FPVCamera->SetRelativeRotation(FRotator(0, 90, 0));
    FPVCamera->SetActive(false);

    // ── PID Init ────────────────────────────────────────────
    PID_Altitude = FPIDController(Altitude_Kp, Altitude_Ki, Altitude_Kd);
    PID_Pitch = FPIDController(Pitch_Kp, Pitch_Ki, Pitch_Kd);
    PID_Roll = FPIDController(Roll_Kp, Roll_Ki, Roll_Kd);
}

void ADronePawn::BeginPlay()
{
    Super::BeginPlay();
    CurrentBattery = MaxBattery;
    CurrentWind = FVector::ZeroVector;
}

void ADronePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (!bCrashed)
    {
        UpdateWind(DeltaTime);
        ApplyDronePhysics(DeltaTime);
        SpinRotors(DeltaTime);
        CheckCrash();
    }
}

// ─────────────────────────────────────────────────────────
void ADronePawn::ApplyDronePhysics(float DeltaTime)
{
    const float Gravity = -980.f;

    // ── Battery ─────────────────────────────────────────────
    if (bIsDead)
    {
        // No thrust when battery dead
        FVector NewPos = GetActorLocation() + Velocity * DeltaTime;
        Velocity += FVector(0, 0, Gravity) * DeltaTime;
        if (NewPos.Z < 0.f) { NewPos.Z = 0.f; Velocity = FVector::ZeroVector; }
        SetActorLocation(NewPos);
        return;
    }

    // Drain battery when throttle is used
    if (InputThrottle > 0.f)
    {
        CurrentBattery -= BatteryDrainRate * InputThrottle * DeltaTime;
        CurrentBattery = FMath::Clamp(CurrentBattery, 0.f, MaxBattery);
    }

    if (CurrentBattery <= 0.f)
    {
        bIsDead = true;
        UE_LOG(LogTemp, Warning, TEXT("Battery Dead! Drone Falling!"));
        return;
    }

    // Battery affects thrust
    float BatteryFactor = CurrentBattery / MaxBattery;
    float Thrust = FMath::Clamp(InputThrottle * MaxThrust * BatteryFactor,
        0.f, MaxThrust);

    // ── Rotation ────────────────────────────────────────────
    float TargetPitch = -InputPitch * MaxTiltAngle;
    float TargetRoll = InputRoll * MaxTiltAngle;
    CurrentYaw += InputYaw * 90.f * DeltaTime;

    FRotator NewRot(TargetPitch, CurrentYaw, TargetRoll);
    SetActorRotation(NewRot);

    // ── Forces ──────────────────────────────────────────────
    FVector Up = GetActorUpVector();
    FVector Forward = GetActorForwardVector();
    FVector Right = GetActorRightVector();

    FVector Accel = Up * (Thrust / Mass)
        + FVector(0, 0, Gravity)
        + Forward * (-InputPitch * 300.f)
        + Right * (InputRoll * 300.f)
        + CurrentWind; // ← Wind effect!

    PreviousVelocity = Velocity;
    Velocity += Accel * DeltaTime;
    Velocity *= FMath::Clamp(1.f - 0.8f * DeltaTime, 0.f, 1.f);

    FVector NewPos = GetActorLocation() + Velocity * DeltaTime;

    if (NewPos.Z < 0.f)
    {
        NewPos.Z = 0.f;
        Velocity.Z = 0.f;
    }

    SetActorLocation(NewPos);
}

// ─────────────────────────────────────────────────────────
void ADronePawn::UpdateWind(float DeltaTime)
{
    WindTimer += DeltaTime;

    if (WindTimer >= WindChangeInterval)
    {
        WindTimer = 0.f;

        // Random wind direction every few seconds
        CurrentWind = FVector(
            FMath::RandRange(-WindStrength, WindStrength),
            FMath::RandRange(-WindStrength, WindStrength),
            FMath::RandRange(-WindStrength * 0.2f, WindStrength * 0.2f)
        );

        UE_LOG(LogTemp, Warning, TEXT("Wind Changed: X=%.1f Y=%.1f Z=%.1f"),
            CurrentWind.X, CurrentWind.Y, CurrentWind.Z);
    }
}

// ─────────────────────────────────────────────────────────
void ADronePawn::CheckCrash()
{
    // Check for sudden velocity change = crash impact
    float ImpactForce = (Velocity - PreviousVelocity).Size();

    if (ImpactForce > 800.f && GetActorLocation().Z < 5.f)
    {
        bCrashed = true;
        Velocity = FVector::ZeroVector;
        UE_LOG(LogTemp, Warning, TEXT("DRONE CRASHED! Impact Force: %.1f"), ImpactForce);
    }
}

// ─────────────────────────────────────────────────────────
void ADronePawn::Recharge()
{
    if (bCrashed) return;

    CurrentBattery = FMath::Clamp(
        CurrentBattery + BatteryRechargeRate,
        0.f, MaxBattery
    );
    bIsDead = false;

    UE_LOG(LogTemp, Warning, TEXT("Battery Recharged: %.1f"), CurrentBattery);
}

// ─────────────────────────────────────────────────────────
void ADronePawn::ToggleFPV()
{
    bIsFPV = !bIsFPV;
    FPVCamera->SetActive(bIsFPV);
    Camera->SetActive(!bIsFPV);
}

// ─────────────────────────────────────────────────────────
void ADronePawn::SpinRotors(float DeltaTime)
{
    float BatteryFactor = CurrentBattery / MaxBattery;
    float Speed = RotorSpinSpeed
        * (0.3f + FMath::Abs(InputThrottle) * 0.7f)
        * BatteryFactor; // Rotors slow with battery!

    RotorAngle = FMath::Fmod(RotorAngle + Speed * DeltaTime, 360.f);

    FRotator CW(0, RotorAngle, 0);
    FRotator CCW(0, -RotorAngle, 0);

    Rotor_FL->SetRelativeRotation(CCW);
    Rotor_FR->SetRelativeRotation(CW);
    Rotor_BL->SetRelativeRotation(CW);
    Rotor_BR->SetRelativeRotation(CCW);
}

// ─────────────────────────────────────────────────────────
// HUD Getters
float ADronePawn::GetSpeed()    const { return Velocity.Size(); }
float ADronePawn::GetAltitude() const { return GetActorLocation().Z; }
float ADronePawn::GetBattery()  const { return CurrentBattery; }
bool  ADronePawn::IsCrashed()   const { return bCrashed; }
FVector ADronePawn::GetWindDirection() const { return CurrentWind; }

// ─────────────────────────────────────────────────────────
void ADronePawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    PlayerInputComponent->BindAxis("Throttle", this, &ADronePawn::OnThrottleUp);
    PlayerInputComponent->BindAxis("Pitch", this, &ADronePawn::OnPitch);
    PlayerInputComponent->BindAxis("Roll", this, &ADronePawn::OnRoll);
    PlayerInputComponent->BindAxis("Yaw", this, &ADronePawn::OnYaw);

    PlayerInputComponent->BindAction("Recharge", IE_Pressed, this, &ADronePawn::Recharge);
    PlayerInputComponent->BindAction("ToggleFPV", IE_Pressed, this, &ADronePawn::ToggleFPV);
}

void ADronePawn::OnThrottleUp(float Val) { InputThrottle = Val; }
void ADronePawn::OnPitch(float Val) { InputPitch = Val; }
void ADronePawn::OnRoll(float Val) { InputRoll = Val; }
void ADronePawn::OnYaw(float Val) { InputYaw = Val; }