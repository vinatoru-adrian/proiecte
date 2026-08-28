# Parking Control System

Sistem centralizat pentru controlul accesului în parcare prin recunoașterea numerelor de înmatriculare (LPR), monitorizare live și administrarea locatarilor și a vehiculelor asociate.

## Motivația proiectului

Am construit acest sistem pentru a automatiza complet controlul de acces într-o parcare rezidențială/comercială, eliminând nevoia de carduri fizice sau operatori manuali la barieră, și pentru a oferi vizibilitate centralizată, în timp real, asupra tuturor evenimentelor de acces.

## Ce face platforma

- **Monitorizare live** — grilă cu feed-urile tuturor camerelor LPR active, actualizată automat, cu ultimul număr detectat și decizia de acces
- **Control automat al barierelor** — pe baza plăcuțelor de înmatriculare înregistrate, cu opțiune de intervenție manuală din interfață
- **Administrare locatari și vehicule** — fiecare locatar poate avea mai multe vehicule asociate și un număr configurabil de locuri de parcare alocate
- **Control de capacitate** — sistemul verifică automat, la fiecare intrare, dacă locatarul mai are locuri disponibile și refuză accesul la depășirea capacității alocate
- **Jurnal complet de evenimente** — fiecare eveniment (autorizat sau refuzat) este înregistrat cu timestamp, cameră și motiv, pentru trasabilitate completă
- **Autentificare și roluri** — Admin / Operator

## Impact

| Proces | Înainte | După |
|---|---|---|
| Control acces parcare | Operator la barieră sau card fizic | Automat, prin recunoaștere plăcuță |
| Înregistrare vehicul nou | Emitere card fizic, configurare manuală | Introducere număr în interfața web, în secunde |
| Monitorizare ocupare | Numărare manuală / sisteme separate | Urmărire automată intrări/ieșiri per locatar |
| Audit acces | Jurnale fizice sau inexistente | Jurnal digital complet, cu timestamp și decizie |

## Detalii tehnice

Sistemul procesează în timp real fiecare eveniment LPR primit de la camere: deduplichează evenimentele duplicate, gestionează ordinea temporală corectă pentru evenimente întârziate, normalizează numărul de înmatriculare indiferent de formatul primit de la cameră și autentifică fiecare cameră individual înainte de a-i accepta evenimentele. Comunicarea cu camerele este securizată împotriva interceptării și manipulării.

Flux de procesare a unui eveniment:
`Eveniment cameră LPR → Autentificare cameră → Extragere număr → Verificare acces → Actualizare stare → Comandă barieră`

## Stack tehnic

Aplicație web full-stack, cu integrare nativă pe camere LPR standard de piață (fără licențe sau echipamente proprietare), procesare de evenimente în timp real și interfață responsive pentru administrare de pe orice dispozitiv.

*Notă: datele afișate (nume, numere de înmatriculare) sunt date demonstrative.*
