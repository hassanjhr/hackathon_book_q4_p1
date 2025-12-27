# Data Model: Navbar & Auth UI Cleanup

**Feature:** 004-navbar-auth-ui-cleanup
**Date:** 2025-12-17
**Phase:** 1 (Design)

---

## Overview

This feature is **frontend-only** with **no backend data persistence**. Therefore, there is **no data model** in the traditional sense (no database tables, no API contracts, no state management beyond local React state).

---

## UI State Models

### 1. UserIconNavbarItem Component State

**Purpose:** Track navbar user icon interaction state

```typescript
interface UserIconNavbarItemProps {
  // Configuration props (from docusaurus.config.ts)
  mobile?: boolean;         // Responsive mode flag
  authUrl?: string;         // Destination URL (default: '/auth')

  // No internal state required - stateless component
}
```

**State Management:** None (stateless functional component)

---

### 2. Auth Choice Page State

**Purpose:** Navigation hub for login/signup

**File:** `src/pages/auth.tsx`

**State:** None required (static navigation page)

**Data Flow:**
```
User clicks "Sign In" → Navigate to /login
User clicks "Sign Up" → Navigate to /signup
```

---

### 3. Login Page State

**Purpose:** Collect login credentials (non-functional demo)

**File:** `src/pages/login.tsx`

```typescript
interface LoginFormData {
  email: string;
  password: string;
}

interface LoginFormState {
  formData: LoginFormData;
  isSubmitting: boolean;     // UI feedback during "submit"
  error: string | null;      // Validation error messages
}
```

**State Management:** React `useState` hook

**Validation Rules:**
- Email: Required, must match email regex pattern
- Password: Required, minimum 8 characters

**Data Lifecycle:**
```
1. User types → Update formData state
2. User submits → Validate locally
3. If valid → Show "Coming soon" message (no API call)
4. If invalid → Display error messages
5. No data persistence → State resets on page reload
```

---

### 4. Signup Page State

**Purpose:** Collect signup information (non-functional demo)

**File:** `src/pages/signup.tsx`

```typescript
interface SignupFormData {
  name: string;
  email: string;
  password: string;
}

interface SignupFormState {
  formData: SignupFormData;
  isSubmitting: boolean;
  errors: Partial<Record<keyof SignupFormData, string>>;
}
```

**State Management:** React `useState` hook

**Validation Rules:**
- Name: Required, minimum 2 characters
- Email: Required, must match email regex pattern
- Password: Required, minimum 8 characters

**Data Lifecycle:**
```
1. User types → Update formData state
2. User submits → Validate locally
3. If valid → Show "Coming soon" message
4. If invalid → Display field-specific errors
5. No data persistence → State resets on page reload
```

---

## Configuration Data (Static)

### Navbar Configuration

**Location:** `my-website/docusaurus.config.ts`

```typescript
interface NavbarConfig {
  title: string;
  logo: {
    alt: string;
    src: string;
  };
  items: NavbarItem[];
}

interface CustomUserIconItem {
  type: 'custom-userIcon';
  position: 'left' | 'right';
  authUrl?: string;
  mobile?: boolean;
}
```

**Data Source:** Static configuration file (no runtime changes)

---

## No Backend Entities

**Entities NOT Created:**
- ❌ User accounts
- ❌ Authentication tokens
- ❌ Session data
- ❌ Login history
- ❌ User preferences

**Rationale:** This is a **UI-only prototype** demonstrating the auth flow interface without backend implementation.

---

## Type Definitions (TypeScript Interfaces)

### Shared Types

**File:** `src/types/auth.ts` (optional, can be inline)

```typescript
export interface AuthFormData {
  email: string;
  password: string;
}

export interface SignupFormData extends AuthFormData {
  name: string;
}

export type FormValidationError = string | null;

export interface FormState<T> {
  data: T;
  isSubmitting: boolean;
  errors: Partial<Record<keyof T, string>>;
}
```

---

## Component Prop Interfaces

### UserIconNavbarItem Props

```typescript
interface UserIconNavbarItemProps {
  mobile?: boolean;
  authUrl?: string;
}
```

### Page Layout Props

```typescript
interface AuthPageLayoutProps {
  children: React.ReactNode;
  title: string;
  description?: string;
}
```

---

## No API Contracts Required

**Why:**
- No backend API endpoints
- No data persistence
- No network requests
- Frontend-only validation

**Future Integration Points:**
When backend is implemented, these endpoints would be added:
- `POST /api/auth/login` - Authenticate user
- `POST /api/auth/signup` - Create new account
- `POST /api/auth/logout` - End session
- `GET /api/auth/me` - Get current user

**(Not implemented in this phase)**

---

## State Persistence

**Local Storage:** None

**Session Storage:** None

**Cookies:** None

**Rationale:** All state is ephemeral (resets on page reload). No need for persistence in demo phase.

---

## Data Flow Diagram

```
┌─────────────────────────────────────────┐
│         Docusaurus Navbar               │
│  [Logo] [Learning]  [🌐] [GitHub] [👤] │
└────────────────┬────────────────────────┘
                 │ Click user icon
                 ▼
         ┌───────────────┐
         │  /auth Page   │
         │  Welcome!     │
         │  [Sign In]    │
         │  [Sign Up]    │
         └───┬───────┬───┘
             │       │
    Click    │       │    Click
    Sign In  │       │    Sign Up
             ▼       ▼
      ┌─────────┐ ┌─────────┐
      │ /login  │ │ /signup │
      │ Page    │ │ Page    │
      └─────────┘ └─────────┘
           │           │
           │ Submit    │ Submit
           ▼           ▼
    "Coming soon"  "Coming soon"
    (No API call)  (No API call)
```

---

## Validation Logic

### Email Validation

```typescript
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;

function validateEmail(email: string): string | null {
  if (!email.trim()) {
    return 'Email is required';
  }
  if (!emailRegex.test(email)) {
    return 'Invalid email format';
  }
  return null;
}
```

### Password Validation

```typescript
function validatePassword(password: string): string | null {
  if (!password) {
    return 'Password is required';
  }
  if (password.length < 8) {
    return 'Password must be at least 8 characters';
  }
  return null;
}
```

### Name Validation

```typescript
function validateName(name: string): string | null {
  if (!name.trim()) {
    return 'Name is required';
  }
  if (name.trim().length < 2) {
    return 'Name must be at least 2 characters';
  }
  return null;
}
```

---

## Summary

**Data Model Scope:** Client-side only (React component state)

**Entities:** None (no backend)

**Persistence:** None (ephemeral state)

**Validation:** Client-side only

**API Contracts:** None required

**Type Safety:** Full TypeScript coverage

**State Management:** React `useState` hooks

**Future Readiness:** Type interfaces designed for easy backend integration

---

## Complexity Assessment

| Aspect | Complexity | Justification |
|--------|-----------|---------------|
| Data Model | MINIMAL | No database, no persistence |
| State Management | SIMPLE | Local React state only |
| Validation | SIMPLE | Basic client-side rules |
| Type Safety | MODERATE | TypeScript interfaces for all data |
| Integration | N/A | No backend integration |

**Overall Complexity:** LOW (frontend-only with no persistence)

---

**Status:** ✅ Data Model Complete
**Next:** contracts/ (none required), quickstart.md
