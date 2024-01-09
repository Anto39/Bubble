typedef enum 
{
		PARAM,
        EAU_FROIDE,
        EAU_TEMPEREE,
        EAU_CARBONATEE,
		EAU_NOTPRESSED
}etypeEau_t;


typedef enum 
{
		HomePage,
		BucketLevelPage,
        SettingsPage,
        AlertPage
}etypePage_t;


typedef enum 
{
        comp_activated,
		comp_deactivated
}ecompresseur_t;


typedef     enum
{
        DDP_activated,
		DDP_deactivated
} eDDP_t;

namespace boutons_t
{
	constexpr uint8_t BOUTON_EAU_FROIDE = 0;
	constexpr uint8_t BOUTON_EAU_TEMP = 1;
	constexpr uint8_t BOUTON_EAU_CARB = 2;
	constexpr uint8_t BOUTON_Settings = 3;
	constexpr uint8_t BOUTON_Alert = 4;
	constexpr uint8_t NO_BOUTON_PRESSED = 5;
};
